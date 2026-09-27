import argparse
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import ant_jetson as tool


class DeployTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()

    def test_toolkit_pack_verify_and_smoke(self):
        bundle = self.root / 'bundle'
        self.assertEqual(tool.pack(bundle)['kind'], 'toolkit-only')
        self.assertEqual(tool.smoke(bundle)['status'], 'toolkit-verified-no-model')
        with self.assertRaises(FileExistsError):
            tool.pack(bundle)

    def test_corruption_extra_files_and_symlinks_rejected(self):
        for scenario in ('corrupt', 'extra', 'symlink'):
            with self.subTest(scenario=scenario):
                bundle = self.root / scenario
                tool.pack(bundle)
                if scenario == 'corrupt':
                    (bundle / 'ant_jetson.py').write_text('bad')
                elif scenario == 'extra':
                    (bundle / 'extra.py').write_text('bad')
                else:
                    (bundle / 'ant_jetson.py').unlink()
                    (bundle / 'ant_jetson.py').symlink_to(Path(tool.__file__))
                with self.assertRaises(ValueError):
                    tool.verify(bundle)

    def test_contract_blocks_assistance_torque_placeholder_and_nonfinite(self):
        original = tool.read_json(Path(__file__).with_name('contract.example.json'))
        path = self.root / 'contract.json'
        path.write_text(json.dumps(original))
        with self.assertRaises(ValueError):
            tool.contract(path)
        original['training_run'] = 'test-run-not-production'
        path.write_text(json.dumps(original))
        tool.contract(path)
        for key, value in [('simulation_assistance', True), ('action_type', 'joint_torque'),
                           ('frame', 'world'), ('max_inference_ms', float('nan'))]:
            path.write_text(json.dumps(dict(original, **{key: value})))
            with self.subTest(key=key), self.assertRaises(ValueError):
                tool.contract(path)
        row = [{'observation': [0] * 8, 'expected_action': [0, 0, float('inf')]}]
        path.write_text(json.dumps(row))
        with self.assertRaises(ValueError):
            tool.samples(path, original)

    def test_partial_model_arguments_fail_before_output_creation(self):
        with self.assertRaises(ValueError):
            tool.pack(self.root / 'bundle', model='policy.onnx')
        self.assertFalse((self.root / 'bundle').exists())

    def test_activate_and_rollback_leave_releases_intact(self):
        root = self.root / 'ant-a2'
        for release in ('r1', 'r2'):
            tool.pack(root / 'releases' / release)
        tool.activate(root, 'r1')
        tool.activate(root, 'r2')
        self.assertEqual((root / 'current').resolve(), root / 'releases/r2')
        tool.activate(root, 'r1')
        self.assertEqual((root / 'current').resolve(), root / 'releases/r1')
        self.assertTrue((root / 'releases/r2').exists())
        (root / 'releases/r2/ant_jetson.py').write_text('corrupt')
        with self.assertRaises(ValueError):
            tool.activate(root, 'r2')
        self.assertEqual((root / 'current').resolve(), root / 'releases/r1')

    def test_smoke_failure_does_not_activate(self):
        root = self.root / 'ant-a2'
        tool.pack(root / 'releases/r1')
        with patch.object(tool.subprocess, 'run', side_effect=subprocess.CalledProcessError(1, 'smoke')):
            with self.assertRaises(subprocess.CalledProcessError):
                tool.activate(root, 'r1')
        self.assertFalse((root / 'current').exists())

    def test_deploy_defaults_to_no_network_and_quotes_remote_argv(self):
        tool.pack(self.root / 'bundle')
        args = argparse.Namespace(bundle=str(self.root / 'bundle'), host='user@jetson.local',
                                  root='/home/user/ant-a2', python='/usr/bin/python3',
                                  provider='CPUExecutionProvider', execute=False)
        with patch.object(tool.subprocess, 'run') as run, contextlib.redirect_stdout(io.StringIO()):
            result = tool.deploy(args)
        run.assert_not_called()
        self.assertFalse(result['executed'])
        args.execute = True
        with patch.object(tool.subprocess, 'run') as run, contextlib.redirect_stdout(io.StringIO()):
            tool.deploy(args)
        self.assertEqual(run.call_count, 5)
        self.assertIn('StrictHostKeyChecking=yes', run.call_args_list[0].args[0])
        self.assertIn('activate', run.call_args_list[-1].args[0][-1])
        for host, root in [('user@host;whoami', args.root), ('-oProxyCommand=x', args.root),
                           (args.host, '/'), (args.host, '/home/user/../ant-a2')]:
            with self.assertRaises(ValueError):
                tool.remote_target(host, root)

    def test_deploy_stops_after_transfer_failure(self):
        tool.pack(self.root / 'bundle')
        args = argparse.Namespace(bundle=str(self.root / 'bundle'), host='user@jetson.local',
                                  root='/home/user/ant-a2', python='/usr/bin/python3',
                                  provider='CPUExecutionProvider', execute=True)
        with patch.object(tool.subprocess, 'run', side_effect=[None, None, subprocess.CalledProcessError(1, 'scp')]) as run:
            with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(subprocess.CalledProcessError):
                tool.deploy(args)
        self.assertEqual(run.call_count, 3)

    def test_lock_and_path_escape_rejected(self):
        root = self.root / 'ant-a2'
        tool.pack(root / 'releases/r1')
        with self.assertRaises(ValueError):
            tool.activate(root, '../bad')
        (root / '.activation-lock').write_text('busy')
        with self.assertRaises(FileExistsError):
            tool.activate(root, 'r1')
        self.assertFalse((root / 'current').exists())

    def test_setup_is_offline_hash_locked_and_preserves_existing_venv(self):
        lock = self.root / 'requirements.lock'; lock.write_text('')
        wheels = self.root / 'wheels'; wheels.mkdir()
        env = self.root / 'venv'
        with patch.object(tool.subprocess, 'run') as run:
            tool.setup(env, lock, wheels)
        command = run.call_args_list[-1].args[0]
        for arg in ('--no-index', '--require-hashes', '--only-binary=:all:'):
            self.assertIn(arg, command)
        env.mkdir()
        with patch.object(tool.subprocess, 'run') as run, self.assertRaises(ValueError):
            tool.setup(env, lock, wheels)
        run.assert_not_called()

    def test_doctor_rejects_non_jetson_when_required(self):
        with patch.object(tool.platform, 'system', return_value='Darwin'):
            with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(ValueError):
                tool.doctor(require_jetson=True)

    @unittest.skipUnless(all(importlib.util.find_spec(m) for m in ('numpy', 'onnx', 'onnxruntime')),
                         'Optional ONNX integration requires numpy, onnx, onnxruntime')
    def test_real_onnx_golden_replay_and_wrong_output(self):
        import numpy as np
        import onnx
        from onnx import helper, TensorProto, numpy_helper
        c = tool.read_json(Path(__file__).with_name('contract.example.json'))
        c['training_run'] = 'test-fixture-not-a-trained-policy'
        c['max_inference_ms'] = 100
        cp = self.root / 'contract.json'; cp.write_text(json.dumps(c))
        gp = self.root / 'samples.json'
        gp.write_text(json.dumps([{'observation': [1] * 8, 'expected_action': [.08, .08, .08]}]))
        graph = helper.make_graph([helper.make_node('MatMul', ['observation', 'weights'], ['action'])],
                                  'test-only', [helper.make_tensor_value_info('observation', TensorProto.FLOAT, [1, 8])],
                                  [helper.make_tensor_value_info('action', TensorProto.FLOAT, [1, 3])],
                                  [numpy_helper.from_array(np.full((8, 3), .01, dtype=np.float32), 'weights')])
        model = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 13)])
        model.ir_version = 8
        mp = self.root / 'policy.onnx'; onnx.save(model, str(mp))
        bundle = self.root / 'bundle'; tool.pack(bundle, mp, cp, gp)
        self.assertEqual(tool.smoke(bundle)['status'], 'offline-inference-passed')
        gp.write_text(json.dumps([{'observation': [1] * 8, 'expected_action': [0, 0, 0]}]))
        bad = self.root / 'bad'; tool.pack(bad, mp, cp, gp)
        with self.assertRaisesRegex(ValueError, 'differs'):
            tool.smoke(bad)


if __name__ == '__main__':
    unittest.main()
