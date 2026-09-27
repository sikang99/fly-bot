"""Offline camera detection adapter; no robot or camera connection.

Requires an externally supplied, trusted, end-to-end ONNX detector with NMS.
Input: RGB float32 [1,3,H,W] /255. Output: [1,N,6] xyxy,score,class
in letterboxed input pixels. Raw YOLO heads are NOT supported.
"""
import argparse
import json
import math
import time


def detections(rows, *, scale, pad_x, width, fx, cx, camera, threshold=.5):
    if camera not in ('front', 'rear') or not all(math.isfinite(v) for v in (scale, pad_x, width, fx, cx, threshold)):
        raise ValueError('Invalid camera/calibration')
    if scale <= 0 or fx <= 0 or width <= 0 or not 0 <= cx <= width or not 0 <= threshold <= 1:
        raise ValueError('Invalid calibration/threshold')
    result = []
    for row in rows:
        if len(row) != 6 or not all(math.isfinite(float(v)) for v in row):
            raise ValueError('Detector must output finite Nx6 rows')
        x1, y1, x2, y2, score, label = map(float, row)
        if not 0 <= score <= 1 or not label.is_integer():
            raise ValueError('Invalid score/class ID')
        if score < threshold or label not in (0, 2):
            continue
        if x2 <= x1 or y2 <= y1:
            continue
        u = ((x1 + x2) / 2 - pad_x) / scale
        if not 0 <= u <= width:
            continue
        bearing = math.atan2(cx - u, fx) + (math.pi if camera == 'rear' else 0)
        result.append({'bearing': math.atan2(math.sin(bearing), math.cos(bearing)),
                       'confidence': score, 'kind': 'person' if label == 0 else 'car'})
    return sorted(result, key=lambda item: -item['confidence'])[:100]


def infer(model, image, camera, fx, cx, provider):
    import cv2
    import numpy as np
    import onnxruntime as ort
    if provider not in ort.get_available_providers():
        raise ValueError('Requested ONNX provider is unavailable')
    session = ort.InferenceSession(model, providers=[provider]); session.disable_fallback()
    inputs, outputs = session.get_inputs(), session.get_outputs()
    if len(inputs) != 1 or len(outputs) != 1:
        raise ValueError('Expected one input and one NMS output')
    shape = inputs[0].shape
    if len(shape) != 4 or shape[:2] != [1, 3] or not all(type(v) is int and 0 < v <= 2048 for v in shape[2:]) or inputs[0].type != 'tensor(float)':
        raise ValueError('Expected fixed float32 [1,3,H,W], H/W <= 2048')
    frame = cv2.imread(image)
    if frame is None:
        raise ValueError('Cannot read image; supply an undistorted image matching fx/cx calibration')
    height, width = frame.shape[:2]; h, w = shape[2:]
    scale = min(w / width, h / height)
    rw, rh = round(width * scale), round(height * scale)
    left, top = (w - rw) // 2, (h - rh) // 2
    resized = cv2.resize(frame, (rw, rh))
    padded = np.full((h, w, 3), 114, dtype=np.uint8)
    padded[top:top + rh, left:left + rw] = resized
    array = np.ascontiguousarray(padded[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32) / 255
    started = time.perf_counter()
    result = session.run([outputs[0].name], {inputs[0].name: array})[0]
    elapsed = (time.perf_counter() - started) * 1000
    if result.ndim != 3 or result.shape[0] != 1 or result.shape[2] != 6:
        raise ValueError('Expected [1,N,6] NMS output; raw detector heads require an exporter adapter')
    return {'source': 'offline-image-onnx', 'camera': camera, 'inference_ms': elapsed,
            'detections': detections(result[0], scale=scale, pad_x=left, width=width, fx=fx, cx=cx, camera=camera),
            'note': 'No distance or capture timestamp inferred. Fuse with synchronized calibrated LiDAR upstream.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True); parser.add_argument('--image', required=True)
    parser.add_argument('--camera', choices=['front', 'rear'], required=True)
    parser.add_argument('--fx', type=float, required=True); parser.add_argument('--cx', type=float, required=True)
    parser.add_argument('--provider', default='CPUExecutionProvider')
    args = parser.parse_args()
    try:
        print(json.dumps(infer(args.model, args.image, args.camera, args.fx, args.cx, args.provider), indent=2, allow_nan=False))
    except (ValueError, ImportError, RuntimeError) as error:
        parser.exit(1, str(error) + '\n')
