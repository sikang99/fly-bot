import math
import importlib.util
from pathlib import Path
import tempfile
import unittest
from vision import detections, infer


class VisionTests(unittest.TestCase):
    def decode(self, rows, **kwargs):
        return detections(rows, scale=1, pad_x=0, width=640, fx=500, cx=320, camera=kwargs.get('camera', 'front'))

    def test_front_rear_bearing_and_class(self):
        row = [[300, 50, 340, 200, .9, 0]]
        self.assertEqual(self.decode(row)[0]['bearing'], 0)
        self.assertAlmostEqual(abs(self.decode(row, camera='rear')[0]['bearing']), math.pi)
        self.assertEqual(self.decode([[100, 20, 200, 80, .8, 2]])[0]['kind'], 'car')
        self.assertGreater(self.decode([[100, 20, 200, 80, .8, 2]])[0]['bearing'], 0)

    def test_bad_outputs_and_uncertain_classes(self):
        self.assertEqual(self.decode([[0, 0, 100, 100, .2, 0], [0, 0, 100, 100, .9, 1]]), [])
        for row in [[0, 0, 100, 100, float('nan'), 0], [0, 0, 100, 100, 2, 0], [0, 1, 2]]:
            with self.assertRaises(ValueError): self.decode([row])

    def test_letterbox_projection(self):
        result = detections([[150, 10, 190, 90, .9, 0]], scale=.5, pad_x=10, width=640, fx=500, cx=320, camera='front')
        self.assertEqual(result[0]['bearing'], 0)

    @unittest.skipUnless(all(importlib.util.find_spec(m) for m in ('cv2', 'numpy', 'onnx', 'onnxruntime')),
                         'Optional inference test requires cv2/numpy/onnx/onnxruntime')
    def test_onnx_image_pipeline_with_test_only_constant_detector(self):
        import cv2
        import numpy as np
        import onnx
        from onnx import helper, TensorProto, numpy_helper
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            image = root / 'frame.png'; model = root / 'test.onnx'
            cv2.imwrite(str(image), np.zeros((384, 640, 3), dtype=np.uint8))
            constant = numpy_helper.from_array(np.array([[[300, 50, 340, 200, .9, 0]]], dtype=np.float32))
            graph = helper.make_graph([helper.make_node('Constant', [], ['detections'], value=constant)], 'test-only',
                [helper.make_tensor_value_info('image', TensorProto.FLOAT, [1, 3, 384, 640])],
                [helper.make_tensor_value_info('detections', TensorProto.FLOAT, [1, 1, 6])])
            net = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 13)]); net.ir_version = 8
            onnx.save(net, str(model))
            result = infer(str(model), str(image), 'rear', 500, 320, 'CPUExecutionProvider')
            self.assertEqual(result['detections'][0]['kind'], 'person')
            self.assertAlmostEqual(abs(result['detections'][0]['bearing']), math.pi)
