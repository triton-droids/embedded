"""YOLO11 football inference using the newer td_perception_camera model.

Only class 0 (ball) can supply a target. RGB-D frames and pixel outputs use the
same optical convention as humanoid_vision. No scene ground truth is consumed.

device='cpu' runs the ONNX with ONNX Runtime on one CPU thread.
device='gpu' runs <model>-fp16.engine next to the ONNX with TensorRT
(scripts/build_ball_chase_engines.sh builds it on the target Jetson). The
engine must have been built from this exact ONNX (SHA-256 in its .json record).
Pre- and post-processing are identical for both devices.
"""
import ast
import ctypes
import hashlib
import importlib.util
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort


def engine_path(onnx_path):
    onnx_path = Path(onnx_path)
    return onnx_path.with_name(onnx_path.stem + '-fp16.engine')


def _import_tensorrt():
    """JetPack installs TensorRT's Python binding for the system interpreter only."""
    try:
        import tensorrt
        return tensorrt
    except ImportError:
        pass
    version = f'python{sys.version_info.major}.{sys.version_info.minor}'
    for root in (f'/usr/lib/{version}/dist-packages', '/usr/lib/python3/dist-packages'):
        init = Path(root) / 'tensorrt' / '__init__.py'
        if init.exists():
            spec = importlib.util.spec_from_file_location('tensorrt', init,
                                                          submodule_search_locations=[str(init.parent)])
            module = importlib.util.module_from_spec(spec)
            sys.modules['tensorrt'] = module
            spec.loader.exec_module(module)
            return module
    raise ImportError('TensorRT Python binding not found; install python3-libnvinfer (JetPack)')


class _CudaRuntime:
    """The few CUDA runtime calls needed, via ctypes (no pycuda/cuda-python on JetPack)."""

    H2D, D2H = 1, 2

    def __init__(self):
        for name in ('libcudart.so', 'libcudart.so.12', '/usr/local/cuda/lib64/libcudart.so'):
            try:
                self.lib = ctypes.CDLL(name)
                break
            except OSError:
                continue
        else:
            raise OSError('libcudart not found')
        self.lib.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
        self.lib.cudaMallocHost.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
        self.lib.cudaMemcpyAsync.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t,
                                             ctypes.c_int, ctypes.c_void_p]
        self.lib.cudaStreamCreate.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        self.lib.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]
        self.lib.cudaGetErrorString.restype = ctypes.c_char_p

    def check(self, code, what):
        if code != 0:
            raise RuntimeError(f'{what} failed: {self.lib.cudaGetErrorString(code).decode()}')

    def device(self, nbytes):
        p = ctypes.c_void_p()
        self.check(self.lib.cudaMalloc(ctypes.byref(p), nbytes), 'cudaMalloc')
        return p

    def pinned(self, shape):
        p = ctypes.c_void_p()
        nbytes = int(np.prod(shape)) * 4
        self.check(self.lib.cudaMallocHost(ctypes.byref(p), nbytes), 'cudaMallocHost')
        array = np.ctypeslib.as_array(ctypes.cast(p, ctypes.POINTER(ctypes.c_float)), shape=tuple(shape))
        return p, array

    def stream(self):
        s = ctypes.c_void_p()
        self.check(self.lib.cudaStreamCreate(ctypes.byref(s)), 'cudaStreamCreate')
        return s

    def copy(self, dst, src, nbytes, kind, stream):
        self.check(self.lib.cudaMemcpyAsync(dst, src, nbytes, kind, stream), 'cudaMemcpyAsync')

    def sync(self, stream):
        self.check(self.lib.cudaStreamSynchronize(stream), 'cudaStreamSynchronize')


class _TensorRTRunner:
    """One FP16 engine, float32 I/O, pinned host buffers, one stream. Not thread-safe."""

    def __init__(self, engine_file, onnx_file, input_shape, output_shape):
        record = Path(str(engine_file) + '.json')
        if not Path(engine_file).exists() or not record.exists():
            raise FileNotFoundError(
                f'No TensorRT engine {engine_file}; build it on this device with '
                f'scripts/build_ball_chase_engines.sh {Path(onnx_file).stem}')
        sha = hashlib.sha256(Path(onnx_file).read_bytes()).hexdigest()
        if json.loads(record.read_text()).get('onnx_sha256') != sha:
            raise ValueError(f'{engine_file} was built from a different ONNX; rebuild it with '
                             f'scripts/build_ball_chase_engines.sh {Path(onnx_file).stem}')
        trt = _import_tensorrt()
        self.cuda = _CudaRuntime()
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)
        self.engine = self.runtime.deserialize_cuda_engine(Path(engine_file).read_bytes())
        if self.engine is None:
            raise RuntimeError(f'Could not deserialize {engine_file} (other device or TensorRT version?)')
        self.context = self.engine.create_execution_context()
        names = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]
        inputs = [n for n in names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT]
        outputs = [n for n in names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.OUTPUT]
        if len(inputs) != 1 or len(outputs) != 1:
            raise ValueError('Expected one input and one output tensor')
        self.input_name, self.output_name = inputs[0], outputs[0]
        for name, shape in ((self.input_name, input_shape), (self.output_name, output_shape)):
            if list(self.engine.get_tensor_shape(name)) != list(shape):
                raise ValueError(f'Engine tensor {name} shape {list(self.engine.get_tensor_shape(name))} != ONNX {shape}')
            if self.engine.get_tensor_dtype(name) != trt.DataType.FLOAT:
                raise ValueError(f'Engine tensor {name} is not float32')
        self.in_bytes = int(np.prod(input_shape)) * 4
        self.out_bytes = int(np.prod(output_shape)) * 4
        self.d_in, self.d_out = self.cuda.device(self.in_bytes), self.cuda.device(self.out_bytes)
        self.h_in_ptr, self.h_in = self.cuda.pinned(input_shape)
        self.h_out_ptr, self.h_out = self.cuda.pinned(output_shape)
        self.stream = self.cuda.stream()
        self.context.set_tensor_address(self.input_name, self.d_in.value)
        self.context.set_tensor_address(self.output_name, self.d_out.value)

    def run(self, tensor):
        np.copyto(self.h_in, tensor)
        self.cuda.copy(self.d_in, self.h_in_ptr, self.in_bytes, _CudaRuntime.H2D, self.stream)
        if not self.context.execute_async_v3(self.stream.value):
            raise RuntimeError('TensorRT execution failed')
        self.cuda.copy(self.h_out_ptr, self.d_out, self.out_bytes, _CudaRuntime.D2H, self.stream)
        self.cuda.sync(self.stream)
        return self.h_out.copy()


class FootballDetector:
    def __init__(self, model_path=None, confidence=0.30, device='cpu'):
        cv2.setNumThreads(1)
        from .scene import assets_directory
        path=Path(model_path) if model_path else assets_directory()/'td-perception.onnx'
        if device not in ('cpu', 'gpu'):
            raise ValueError("device must be 'cpu' or 'gpu'")
        options=ort.SessionOptions();options.intra_op_num_threads=1;options.inter_op_num_threads=1
        self.session=ort.InferenceSession(str(path),sess_options=options,providers=['CPUExecutionProvider'])
        model_input=self.session.get_inputs()[0]
        if model_input.shape != [1,3,640,640]:raise ValueError('Expected 640x640 YOLO11 RGB input')
        names=ast.literal_eval(self.session.get_modelmeta().custom_metadata_map['names'])
        if names.get(0)!='ball':raise ValueError('Model class 0 is not ball')
        self.input_name=model_input.name
        self.device=device
        self.engine_path=None
        if device == 'gpu':
            self.engine_path=engine_path(path)
            self._runner=_TensorRTRunner(self.engine_path, path, model_input.shape,
                                         self.session.get_outputs()[0].shape)
            self.session=None   # metadata only; inference runs in TensorRT
        self.confidence=float(confidence)
        self.last_confidence=0.0

    def _infer(self, tensor):
        if self.device == 'gpu':
            return self._runner.run(tensor)
        return self.session.run(None,{self.input_name:tensor})[0]

    def detect(self, rgb, depth_m, k):
        self.last_confidence=0.0
        h,w=rgb.shape[:2]
        if depth_m.shape!=(h,w):return None
        scale=min(640/w,640/h)
        nw,nh=round(w*scale),round(h*scale)
        left,top=(640-nw)//2,(640-nh)//2
        image=np.full((640,640,3),114,np.uint8)
        image[top:top+nh,left:left+nw]=cv2.resize(rgb,(nw,nh))
        tensor=np.ascontiguousarray(image.transpose(2,0,1)[None],dtype=np.float32)/255
        prediction=self._infer(tensor)[0].T
        scores=prediction[:,4:]
        keep=(scores.argmax(axis=1)==0)&(scores[:,0]>=self.confidence)
        rows=prediction[keep];conf=scores[keep,0]
        if not len(rows):return None
        boxes=[]
        for cx,cy,bw,bh in rows[:,:4]:
            boxes.append([(float(cx-bw/2)-left)/scale,(float(cy-bh/2)-top)/scale,
                          float(bw)/scale,float(bh)/scale])
        selected=cv2.dnn.NMSBoxes(boxes,conf.tolist(),self.confidence,.45)
        mat=np.asarray(k).reshape(3,3)
        if not np.isfinite(mat).all() or min(mat[0,0],mat[1,1])<=0:return None
        for idx in np.asarray(selected).reshape(-1):
            x,y,bw,bh=boxes[int(idx)]
            point=point_from_box(x+bw/2,y+bh/2,bw,bh,depth_m,mat)
            if point is None:continue
            self.last_confidence=float(conf[idx])
            return point
        return None


def point_from_box(u, v, bw, bh, depth_m, k):
    """(u, v, x, y, z) in camera optical axes from a box center and the aligned depth.

    Median of valid depth (0.1-10 m) in a window of 20 % of the smaller box side
    around the center; None if the center is outside the image, fewer than five
    depth samples are valid, or the intrinsics are invalid.
    """
    h, w = depth_m.shape
    if not (0 <= u < w and 0 <= v < h):
        return None
    mat = np.asarray(k, dtype=float).reshape(3, 3)
    if not np.isfinite(mat).all() or min(mat[0, 0], mat[1, 1]) <= 0:
        return None
    radius = max(1, int(min(bw, bh) * .20))
    x0, x1 = max(0, int(u) - radius), min(w, int(u) + radius + 1)
    y0, y1 = max(0, int(v) - radius), min(h, int(v) + radius + 1)
    samples = depth_m[y0:y1, x0:x1]
    samples = samples[np.isfinite(samples) & (samples > .1) & (samples < 10)]
    if len(samples) < 5:
        return None
    z = float(np.median(samples))
    return float(u), float(v), (u - mat[0, 2]) * z / mat[0, 0], (v - mat[1, 2]) * z / mat[1, 1], z
