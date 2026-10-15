#!/usr/bin/env python3
"""Convert the bundled fixed-640 face model to OpenCV 4.5 ONNX operators.

Requires onnx==1.16.2 and numpy. Keeps weights and the output interface intact.
The source hash restricts these graph-specific rewrites to the tested model.
"""
import hashlib
from pathlib import Path

import numpy as np
import onnx
from onnx import helper, numpy_helper

root = Path(__file__).resolve().parent.parent
source = root / 'humanoid_vision/models/yolov8n-face-lindevs.onnx'
expected = '8d0bfb0c3383c5bd7a78dd24ef79a21e2aa456619b6ab5e53867092d1c7dc414'
if hashlib.sha256(source.read_bytes()).hexdigest() != expected:
    raise ValueError('Source model changed; review the graph before conversion')
model = onnx.load(source)
constants = {x.name: numpy_helper.to_array(x) for x in model.graph.initializer}
nodes = []
for node in model.graph.node:
    if node.op_type == 'Split':
        sizes = constants[node.input[1]].tolist()
        del node.input[1:]
        node.attribute.append(helper.make_attribute('split', sizes))
    if node.op_type == 'Reshape':
        for attribute in list(node.attribute):
            if attribute.name == 'allowzero':
                assert attribute.i == 0
                node.attribute.remove(attribute)
    if node.op_type == 'Resize' and not node.input[1]:
        name = node.name + '_roi'
        node.input[1] = name
        model.graph.initializer.append(numpy_helper.from_array(
            np.array([], dtype=np.float32), name))
    if node.name == '/model.22/Sub':
        # Avoid OpenCV 4.5's constant-first Sub handling.
        name = node.name + '_neg'
        nodes.append(helper.make_node('Neg', [node.input[1]], [name], name=name))
        node = helper.make_node('Add', [name, node.input[0]], node.output,
                                name=node.name)
    nodes.append(node)
del model.graph.node[:]
model.graph.node.extend(nodes)
for index, tensor in enumerate(model.graph.initializer):
    if tensor.name == '/model.22/Constant_12_output_0':
        # Older DNN cannot broadcast [1,8400] over [1,4,8400].
        expanded = np.broadcast_to(numpy_helper.to_array(tensor), (1, 4, 8400)).copy()
        model.graph.initializer[index].CopyFrom(
            numpy_helper.from_array(expanded, tensor.name))
model.opset_import[0].version = 12
onnx.checker.check_model(model)
output = source.with_name('yolov8n-face-lindevs-opencv45.onnx')
onnx.save(model, output)
print(output)
