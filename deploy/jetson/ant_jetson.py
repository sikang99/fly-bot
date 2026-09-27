#!/usr/bin/env python3
"""Offline Jetson deployment tooling. Never opens DDS or sends robot commands."""
import argparse
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import platform
import re
import shlex
import shutil
import subprocess
import sys
import time
import uuid


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def emit(value):
    print(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False))


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def finite(value):
    return type(value) in (float, int) and math.isfinite(value)


def contract(path):
    c = read_json(path)
    require(isinstance(c, dict), 'Contract must be a JSON object')
    require(c.get('schema_version') == 1, 'Unsupported contract version')
    require(c.get('robot') == 'unitree_a2', 'Contract must target unitree_a2')
    require(c.get('action_type') == 'body_velocity', 'Joint/torque policies are not supported')
    require(c.get('frame') == 'body_x_forward_y_left_z_up', 'Wrong frame')
    require(c.get('simulation_assistance') is False, 'Assisted simulation policies cannot pass this gate')
    require(c.get('normalization') == 'embedded_in_onnx', 'Embed preprocessing in ONNX')
    require(isinstance(c.get('training_run'), str) and c['training_run'].strip()
            and 'REPLACE' not in c['training_run'], 'Set a real training_run identifier')
    obs = c.get('observations')
    require(isinstance(obs, list) and 0 < len(obs) <= 4096, 'Missing observation contract')
    require(all(isinstance(o, dict) and isinstance(o.get('name'), str) and o['name']
                and isinstance(o.get('unit'), str) and o['unit'] for o in obs), 'Name and unit required for every observation')
    require(len({o['name'] for o in obs}) == len(obs), 'Duplicate observations')
    require(c.get('outputs') == ['vx_m_s', 'vy_m_s', 'yaw_rad_s'], 'Output order mismatch')
    require(all(isinstance(c.get(k), str) and c[k] for k in ('input_name', 'output_name')), 'Tensor names required')
    require(finite(c.get('max_inference_ms')) and 0 < c['max_inference_ms'] <= 100,
            'max_inference_ms must be in (0,100]')
    require(finite(c.get('atol')) and 0 < c['atol'] <= .01, 'atol must be in (0,0.01]')
    return c


def samples(path, c):
    rows = read_json(path)
    require(isinstance(rows, list) and 1 <= len(rows) <= 10000, 'Provide 1..10000 golden samples')
    for row in rows:
        require(isinstance(row, dict), 'Invalid sample')
        for key, size in [('observation', len(c['observations'])), ('expected_action', 3)]:
            value = row.get(key)
            require(isinstance(value, list) and len(value) == size and all(finite(v) for v in value),
                    'Invalid finite sample vector: ' + key)
    return rows


def pack(output, model=None, metadata=None, golden=None):
    require(all([model, metadata, golden]) or not any([model, metadata, golden]),
            'Supply --model, --contract and --samples together, or omit all for toolkit-only')
    if model:
        require(Path(model).suffix == '.onnx' and Path(model).stat().st_size > 0, 'Expected nonempty ONNX file')
        c = contract(metadata)
        samples(golden, c)
    target = Path(output).absolute()
    target.mkdir(parents=True, exist_ok=False)
    files = {'ant_jetson.py': Path(__file__)}
    vision = Path(__file__).with_name('vision.py')
    if vision.is_file():
        files['vision.py'] = vision
    if model:
        files.update({'policy.onnx': Path(model), 'contract.json': Path(metadata), 'samples.json': Path(golden)})
    for name, source in files.items():
        shutil.copyfile(source, target / name)
    manifest = {'schema_version': 1, 'kind': 'offline-policy' if model else 'toolkit-only',
                'created_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                'hardware_enabled': False,
                'files': {name: digest(target / name) for name in files}}
    (target / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    return verify(target)


def verify(directory):
    root = Path(directory).resolve()
    manifest = read_json(root / 'manifest.json')
    require(isinstance(manifest, dict), 'Manifest must be a JSON object')
    require(manifest.get('schema_version') == 1 and manifest.get('hardware_enabled') is False,
            'Unsupported or hardware-enabled bundle')
    kind = manifest.get('kind')
    require(kind in ('toolkit-only', 'offline-policy'), 'Unknown bundle kind')
    expected = {'ant_jetson.py'} if kind == 'toolkit-only' else {'ant_jetson.py', 'policy.onnx', 'contract.json', 'samples.json'}
    if isinstance(manifest.get('files'), dict) and 'vision.py' in manifest['files']:
        expected.add('vision.py')
    require(isinstance(manifest.get('files'), dict) and set(manifest['files']) == expected, 'Unexpected bundle files')
    require({p.name for p in root.iterdir()} == expected | {'manifest.json'}, 'Bundle has untracked files')
    for name in expected | {'manifest.json'}:
        path = root / name
        require(not path.is_symlink() and path.is_file(), 'Only regular bundle files allowed')
        if name != 'manifest.json':
            require(digest(path) == manifest['files'][name], 'Checksum mismatch: ' + name)
    if kind == 'offline-policy':
        samples(root / 'samples.json', contract(root / 'contract.json'))
    return manifest


def doctor(require_jetson=False):
    model_path = Path('/proc/device-tree/model')
    model = model_path.read_text().strip('\x00\n') if model_path.exists() else None
    l4t = Path('/etc/nv_tegra_release')
    report = {'system': platform.system(), 'arch': platform.machine(), 'python': platform.python_version(),
              'device_model': model, 'l4t': l4t.read_text().strip() if l4t.exists() else None,
              'interfaces': sorted(p.name for p in Path('/sys/class/net').glob('*')),
              'robot_connection_tested': False, 'hardware_enabled': False}
    for name in ('numpy', 'onnxruntime', 'onnxruntime-gpu', 'tensorrt', 'unitree_sdk2py'):
        try:
            report[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            report[name] = None
    if shutil.which('dpkg-query'):
        result = subprocess.run(['dpkg-query', '-W', '-f=${Version}', 'nvidia-jetpack'],
                                capture_output=True, text=True, timeout=10)
        report['jetpack_package'] = result.stdout if result.returncode == 0 else None
    report['jetson_detected'] = platform.system() == 'Linux' and platform.machine() in ('aarch64', 'arm64') and l4t.exists()
    emit(report)
    require(not require_jetson or report['jetson_detected'], 'Target is not a detected Jetson/L4T installation')
    return report


def smoke(directory, provider='CPUExecutionProvider'):
    root = Path(directory).resolve()
    manifest = verify(root)
    if manifest['kind'] == 'toolkit-only':
        return {'status': 'toolkit-verified-no-model', 'hardware_enabled': False}
    c = contract(root / 'contract.json')
    rows = samples(root / 'samples.json', c)
    try:
        import numpy as np
        import onnxruntime as ort
    except ImportError as error:
        raise ValueError('Install a JetPack/Python-compatible numpy + ONNX Runtime in the selected venv') from error
    require(provider in ort.get_available_providers(), 'Requested provider unavailable: ' + provider)
    session = ort.InferenceSession(str(root / 'policy.onnx'), providers=[provider])
    session.disable_fallback()
    ins, outs = session.get_inputs(), session.get_outputs()
    require(len(ins) == len(outs) == 1, 'Only a stateless single-input/single-output model is supported')
    require(ins[0].name == c['input_name'] and ins[0].shape == [1, len(c['observations'])]
            and ins[0].type == 'tensor(float)', 'Input must be fixed [1,N] float32 with contract name')
    require(outs[0].name == c['output_name'] and outs[0].shape == [1, 3]
            and outs[0].type == 'tensor(float)', 'Output must be fixed [1,3] float32 with contract name')
    latency = []
    for row in rows:
        array = np.asarray([row['observation']], dtype=np.float32)
        require(bool(np.isfinite(array).all()), 'Observation overflows float32')
        feeds = {c['input_name']: array}
        session.run([c['output_name']], feeds)  # Warm-up; not counted as steady-state latency.
        for _ in range(10):
            start = time.perf_counter()
            result = session.run([c['output_name']], feeds)[0]
            latency.append((time.perf_counter() - start) * 1000)
            require(result.shape == (1, 3) and bool(np.isfinite(result).all()), 'Nonfinite or malformed action')
            require(bool(np.allclose(result[0], row['expected_action'], atol=c['atol'], rtol=0)),
                    'ONNX output differs from training golden action')
    require(max(latency) <= c['max_inference_ms'], 'Inference latency exceeds contract budget')
    return {'status': 'offline-inference-passed', 'samples': len(rows), 'runs': len(latency),
            'max_ms': max(latency), 'mean_ms': sum(latency) / len(latency),
            'requested_provider': provider, 'session_providers': session.get_providers(),
            'hardware_enabled': False, 'note': 'Offline golden replay only; no sensor or robot connection.'}


def activate(root, release, provider='CPUExecutionProvider', python=None):
    root = Path(root).resolve()
    require(re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,80}', release), 'Invalid release ID')
    target = root / 'releases' / release
    require(not target.is_symlink() and target.resolve().parent == root / 'releases', 'Invalid release path')
    verify(target)
    # Use the exact same interpreter for preactivation and subsequent replay.
    subprocess.run([python or sys.executable, '-B', str(target / 'ant_jetson.py'),
                    'smoke', '--bundle', str(target), '--provider', provider], check=True, timeout=180)
    current = root / 'current'
    require(not current.exists() or current.is_symlink(), 'current must be a symlink; refusing overwrite')
    # Serialise activations and keep the previous directory untouched for explicit rollback.
    lock = root / '.activation-lock'
    fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    pending = root / ('.current-' + uuid.uuid4().hex)
    try:
        os.close(fd)
        pending.symlink_to(Path('releases') / release)
        os.replace(pending, current)
    finally:
        if pending.is_symlink():
            pending.unlink()
        lock.unlink()
    return {'active_release': release, 'hardware_enabled': False, 'service_started': False}


def remote_target(host, root):
    require(bool(re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]*@[A-Za-z0-9][A-Za-z0-9.-]*', host)),
            'Use user@hostname or user@IPv4; configure ports/keys in ~/.ssh/config')
    require(bool(re.fullmatch(r'/(?:[A-Za-z0-9_-]+/)+[A-Za-z0-9_-]+', root))
            and root.endswith('/ant-a2'), 'Remote root must be an absolute dedicated path ending /ant-a2')
    return host, root


def deploy(args):
    host, root = remote_target(args.host, args.root)
    require(re.fullmatch(r'/[A-Za-z0-9_./-]+', args.python) is not None and '..' not in args.python.split('/'),
            'Remote --python must be an absolute executable path')
    bundle = Path(args.bundle).resolve()
    verify(bundle)
    release = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()) + '-' + uuid.uuid4().hex[:8]
    destination = root + '/releases/' + release
    options = ['-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=10']
    def ssh(argv):
        return ['ssh', *options, host, shlex.join(argv)]
    commands = [
        ssh(['mkdir', '-p', root + '/releases']),
        ssh(['mkdir', destination]),
        ['scp', *options, *[str(p) for p in sorted(bundle.iterdir())], host + ':' + destination + '/'],
        ssh([args.python, '-B', destination + '/ant_jetson.py', 'doctor', '--require-jetson']),
        ssh([args.python, '-B', destination + '/ant_jetson.py', 'activate', '--root', root,
             '--release', release, '--provider', args.provider]),
    ]
    for command in commands:
        print(shlex.join(command), flush=True)
        if args.execute:
            subprocess.run(command, check=True, timeout=600)
    return {'executed': args.execute, 'release': release, 'hardware_enabled': False,
            'note': 'No SSH connection unless --execute. Existing releases are retained.'}


def setup(venv_path, requirements, wheelhouse):
    target = Path(venv_path).absolute()
    require(not target.exists(), 'Use a new versioned venv; existing environment is never modified')
    require(Path(requirements).is_file() and Path(wheelhouse).is_dir(), 'Need locked requirements and wheelhouse')
    subprocess.run([sys.executable, '-m', 'venv', str(target)], check=True)
    subprocess.run([str(target / 'bin/python'), '-m', 'pip', 'install', '--no-index',
                    '--find-links', str(Path(wheelhouse).resolve()), '--only-binary=:all:',
                    '--require-hashes', '-r', str(Path(requirements).resolve())], check=True, timeout=600)
    return {'venv': str(target), 'note': 'Dependencies installed offline; no service or robot connection.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('doctor'); p.add_argument('--require-jetson', action='store_true')
    p = sub.add_parser('pack'); p.add_argument('--out', required=True)
    for name in ('model', 'contract', 'samples'): p.add_argument('--' + name)
    for name in ('verify', 'smoke'):
        p = sub.add_parser(name); p.add_argument('--bundle', required=True)
        if name == 'smoke': p.add_argument('--provider', default='CPUExecutionProvider')
    p = sub.add_parser('deploy'); p.add_argument('--bundle', required=True)
    p.add_argument('--host', required=True); p.add_argument('--root', required=True)
    p.add_argument('--python', default='/usr/bin/python3'); p.add_argument('--provider', default='CPUExecutionProvider')
    p.add_argument('--execute', action='store_true')
    p = sub.add_parser('activate'); p.add_argument('--root', required=True); p.add_argument('--release', required=True)
    p.add_argument('--provider', default='CPUExecutionProvider')
    p = sub.add_parser('setup'); p.add_argument('--venv', required=True)
    p.add_argument('--requirements', required=True); p.add_argument('--wheelhouse', required=True)
    args = parser.parse_args()
    if args.command == 'doctor': doctor(args.require_jetson)
    elif args.command == 'pack': emit(pack(args.out, args.model, args.contract, args.samples))
    elif args.command == 'verify': emit(verify(args.bundle))
    elif args.command == 'smoke': emit(smoke(args.bundle, args.provider))
    elif args.command == 'activate': emit(activate(args.root, args.release, args.provider))
    elif args.command == 'deploy': emit(deploy(args))
    elif args.command == 'setup': emit(setup(args.venv, args.requirements, args.wheelhouse))


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, subprocess.SubprocessError, RuntimeError) as error:
        print('ERROR: ' + str(error), file=sys.stderr)
        sys.exit(1)
