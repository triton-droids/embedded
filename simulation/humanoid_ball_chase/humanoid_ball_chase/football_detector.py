"""YOLO11 football inference using the newer td_perception_camera model.

Only class 0 (ball) can supply a target. RGB-D frames and pixel outputs use the
same optical convention as humanoid_vision. No scene ground truth is consumed.
"""
import ast
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort


class FootballDetector:
    def __init__(self, model_path=None, confidence=0.30):
        cv2.setNumThreads(1)
        from .scene import assets_directory
        path=Path(model_path) if model_path else assets_directory()/'td-perception.onnx'
        options=ort.SessionOptions();options.intra_op_num_threads=1;options.inter_op_num_threads=1
        self.session=ort.InferenceSession(str(path),sess_options=options,providers=['CPUExecutionProvider'])
        model_input=self.session.get_inputs()[0]
        if model_input.shape != [1,3,640,640]:raise ValueError('Expected 640x640 YOLO11 RGB input')
        names=ast.literal_eval(self.session.get_modelmeta().custom_metadata_map['names'])
        if names.get(0)!='ball':raise ValueError('Model class 0 is not ball')
        self.input_name=model_input.name
        self.confidence=float(confidence)
        self.last_confidence=0.0

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
        prediction=self.session.run(None,{self.input_name:tensor})[0][0].T
        scores=prediction[:,4:]
        keep=(scores.argmax(axis=1)==0)&(scores[:,0]>=self.confidence)
        rows=prediction[keep];conf=scores[keep,0]
        if not len(rows):return None
        boxes=[]
        for cx,cy,bw,bh in rows[:,:4]:
            boxes.append([(float(cx-bw/2)-left)/scale,(float(cy-bh/2)-top)/scale,
                          float(bw)/scale,float(bh)/scale])
        selected=cv2.dnn.NMSBoxes(boxes,conf.tolist(),self.confidence,.45)
        for idx in np.asarray(selected).reshape(-1):
            x,y,bw,bh=boxes[int(idx)]
            u,v=x+bw/2,y+bh/2
            if not (0<=u<w and 0<=v<h):continue
            radius=max(1,int(min(bw,bh)*.20))
            x0,x1=max(0,int(u)-radius),min(w,int(u)+radius+1)
            y0,y1=max(0,int(v)-radius),min(h,int(v)+radius+1)
            samples=depth_m[y0:y1,x0:x1]
            samples=samples[np.isfinite(samples)&(samples>.1)&(samples<10)]
            if len(samples)<5:continue
            z=float(np.median(samples));mat=np.asarray(k).reshape(3,3)
            if not np.isfinite(mat).all() or min(mat[0,0],mat[1,1])<=0:return None
            self.last_confidence=float(conf[idx])
            return float(u),float(v),(u-mat[0,2])*z/mat[0,0],(v-mat[1,2])*z/mat[1,1],z
        return None
