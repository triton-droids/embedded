#!/usr/bin/env python3

import argparse

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('model')
    parser.add_argument('input_image')
    parser.add_argument('output_image')
    args = parser.parse_args()

    image = cv2.imread(args.input_image)
    if image is None:
        raise RuntimeError(f'Failed to read image: {args.input_image}')

    input_size = 640
    height, width = image.shape[:2]
    scale = min(input_size / width, input_size / height)
    resized_width = int(width * scale)
    resized_height = int(height * scale)
    resized = cv2.resize(image, (resized_width, resized_height))

    pad_x = (input_size - resized_width) // 2
    pad_y = (input_size - resized_height) // 2
    padded = np.full((input_size, input_size, 3), 114, dtype=np.uint8)
    padded[
        pad_y:pad_y + resized_height,
        pad_x:pad_x + resized_width,
    ] = resized

    blob = cv2.dnn.blobFromImage(
        padded,
        scalefactor=1.0 / 255.0,
        size=(input_size, input_size),
        swapRB=True,
    )

    net = cv2.dnn.readNetFromONNX(args.model)
    net.setInput(blob)
    output = net.forward()
    print(f'opencv={cv2.__version__} output_shape={output.shape}')

    predictions = output[0]
    if predictions.shape[0] < predictions.shape[1]:
        predictions = predictions.T

    boxes = []
    confidences = []

    for prediction in predictions:
        confidence = float(np.max(prediction[4:]))
        if confidence < 0.25:
            continue

        center_x = (float(prediction[0]) - pad_x) / scale
        center_y = (float(prediction[1]) - pad_y) / scale
        box_width = float(prediction[2]) / scale
        box_height = float(prediction[3]) / scale

        left = max(0, int(center_x - box_width * 0.5))
        top = max(0, int(center_y - box_height * 0.5))
        right = min(width, int(center_x + box_width * 0.5))
        bottom = min(height, int(center_y + box_height * 0.5))

        if right <= left or bottom <= top:
            continue

        boxes.append([left, top, right - left, bottom - top])
        confidences.append(confidence)

    kept = cv2.dnn.NMSBoxes(boxes, confidences, 0.25, 0.45)
    print(f'detections={len(kept)}')

    for index in np.asarray(kept).reshape(-1):
        left, top, box_width, box_height = boxes[int(index)]
        confidence = confidences[int(index)]
        print(
            f'confidence={confidence:.6f} '
            f'box={left},{top},{box_width},{box_height}'
        )
        cv2.rectangle(
            image,
            (left, top),
            (left + box_width, top + box_height),
            (0, 255, 0),
            2,
        )

    if not cv2.imwrite(args.output_image, image):
        raise RuntimeError(f'Failed to write image: {args.output_image}')

    if len(kept) == 0:
        raise RuntimeError('No faces detected')


if __name__ == '__main__':
    main()
