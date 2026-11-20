#!/usr/bin/env python3
"""Prepend GPU preprocessing to a YOLO ONNX so TensorRT takes letterboxed BGR uint8 directly.

The new input is uint8 [1, H, W, 3] (OpenCV BGR, HWC). The added nodes reproduce
cv::dnn::blobFromImage(image, 1/255, swapRB=true): Cast -> Transpose to NCHW ->
Gather channels (B,G,R) -> (R,G,B) -> Mul 1/255. Usage: make_u8_input_onnx.py IN.onnx OUT.onnx
"""
import sys

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper


def main(src, dst):
    model = onnx.load(src)
    graph = model.graph
    if len(graph.input) != 1:
        raise SystemExit('expected exactly one model input')
    old = graph.input[0]
    dims = [d.dim_value for d in old.type.tensor_type.shape.dim]
    if len(dims) != 4 or dims[0] != 1 or dims[1] != 3 or 0 in dims:
        raise SystemExit(f'expected static [1,3,H,W] input, got {dims}')
    if old.type.tensor_type.elem_type != TensorProto.FLOAT:
        raise SystemExit('expected a float32 input')
    _, _, h, w = dims
    name = old.name
    graph.initializer.extend([
        numpy_helper.from_array(np.array([2, 1, 0], dtype=np.int64), 'pre_bgr_to_rgb'),
        numpy_helper.from_array(np.array(1.0 / 255.0, dtype=np.float32), 'pre_scale'),
    ])
    pre = [
        helper.make_node('Cast', ['images_bgr_u8'], ['pre_f32'], to=TensorProto.FLOAT, name='pre_cast'),
        helper.make_node('Transpose', ['pre_f32'], ['pre_nchw'], perm=[0, 3, 1, 2], name='pre_nchw'),
        helper.make_node('Gather', ['pre_nchw', 'pre_bgr_to_rgb'], ['pre_rgb'], axis=1, name='pre_rgb'),
        helper.make_node('Mul', ['pre_rgb', 'pre_scale'], [name], name='pre_scale'),
    ]
    nodes = pre + list(graph.node)
    del graph.node[:]
    graph.node.extend(nodes)
    graph.input.remove(old)
    graph.input.insert(0, helper.make_tensor_value_info('images_bgr_u8', TensorProto.UINT8, [1, h, w, 3]))
    onnx.checker.check_model(model)
    onnx.save(model, dst)
    print(f'{dst}: input images_bgr_u8 uint8 [1,{h},{w},3] -> {name} float32 [1,3,{h},{w}]')


if __name__ == '__main__':
    main(*sys.argv[1:3])
