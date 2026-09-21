"""Read-only by default; supervised launch requires explicit, verified gates.

No Isaac imports. Only the child created here may be terminated. Environment
redirection controls cooperative writes; it is not a filesystem sandbox.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import ipaddress
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

from .paths import confined_path, ensure_directory, ensure_private_file, read_json, atomic_replace_json

SERVER_ROOT = Path('/mnt/DATA/workspace/ws_minho/mro_1')
REQUIRED_PATH_ENV = {
    'HOME', 'XDG_CONFIG_HOME', 'XDG_CACHE_HOME', 'XDG_DATA_HOME',
    'XDG_RUNTIME_DIR', 'XDG_STATE_HOME', 'OMNI_CONFIG_PATH', 'TMPDIR', 'TMP',
    'TEMP', 'CUDA_CACHE_PATH', '__GL_SHADER_DISK_CACHE_PATH', 'TORCH_HOME',
    'HF_HOME', 'NUMBA_CACHE_DIR', 'MPLCONFIGDIR', 'PIP_CACHE_DIR',
    'TRITON_CACHE_DIR', 'PYTHONUSERBASE', 'ROS_HOME', 'ROS_LOG_DIR', 'HISTFILE',
    'MRO_ROOT', 'ISAAC_PATH',
}
ALLOWED_SCALAR_ENV = {'PYTHONDONTWRITEBYTECODE', 'PYTHONNOUSERSITE'}


def build_plan(root) -> dict:
    root = Path(root).resolve(strict=True)
    policy = read_json(root, 'config/launch_policy.json')
    config = read_json(root, 'config/launch_config.json')
    prepared = read_json(root, 'config/prepared_kit_args.json')
    env = read_json(root, 'config/runtime_environment.json')
    if set(env) != REQUIRED_PATH_ENV | ALLOWED_SCALAR_ENV:
        raise ValueError('Unexpected or missing runtime environment keys')
    for name in REQUIRED_PATH_ENV:
        if not isinstance(env[name], str) or not Path(env[name]).is_absolute():
            raise ValueError(f'{name} must be an absolute workspace path')
        confined_path(root, env[name], require_exists=True)
    if Path(env['MRO_ROOT']) != root or Path(env['ISAAC_PATH']) != root / 'issacsim':
        raise ValueError('Runtime environment belongs to another workspace')
    if any(env[k] != '1' for k in ALLOWED_SCALAR_ENV):
        raise ValueError('Python isolation flags must remain enabled')
    for relative in ['config/user.config.json', 'runtime/config/omniverse/omniverse.toml']:
        ensure_private_file(root, relative)
    omni_lines = [line.strip() for line in
                  (root/'runtime/config/omniverse/omniverse.toml').read_text().splitlines()
                  if line.strip() and not line.lstrip().startswith('#')]
    approved_lines = ['[paths]'] + [f'{kind}_root = "{root}/runtime/{kind}"'
                                     for kind in ['data', 'cache', 'logs']]
    # Deliberately accept only our tiny reviewed TOML, including on Python 3.10.
    # A comment containing an approved path cannot validate a different value.
    if omni_lines != approved_lines:
        raise ValueError('Omniverse path configuration differs from approved contents')
    expected_args = [
        '--no-window', '--no-ros-env', '--portable-root', str(root/'runtime/portable'),
        f'--/app/userConfigPath={root}/config/user.config.json',
        f'--/log/file={root}/runtime/logs/isaac-sim.log',
        '--/renderer/multiGpu/enabled=false',
    ]
    if prepared.get('args') != expected_args:
        raise ValueError('Prepared Kit arguments changed; review required')
    if config.get('schema_version') != 1 or config.get('mode') != 'streaming':
        raise ValueError('Only the reviewed streaming configuration is supported')
    cwd = confined_path(root, config['cwd'])
    if not cwd.is_relative_to(root/'outputs'):
        raise ValueError('CWD-based recorder output must stay below outputs')
    ensure_private_file(root, 'issacsim/isaac-sim.streaming.sh')
    ensure_private_file(root, 'issacsim/kit/kit')
    ensure_private_file(root, 'issacsim/apps/isaacsim.exp.full.streaming.kit')
    gpu = config['gpu']
    streaming = config['streaming']
    blockers = []
    if policy.get('simulation_launch_allowed') is not True:
        blockers.append('User launch pause remains active (launch_policy=false)')
    if prepared.get('execute_now') is not True:
        blockers.append('Prepared arguments are not released for execution')
    for name in ['nvidia_index', 'vulkan_index', 'cuda_index']:
        value = gpu.get(name)
        if type(value) is not int or value < 0:
            blockers.append(f'GPU {name} is not selected')
    if gpu.get('mapping_verified') is not True:
        blockers.append('NVIDIA/Vulkan/CUDA device mapping has not been verified')
    if policy.get('gpu_selection') != gpu.get('nvidia_index') or policy.get('gpu_selection') is None:
        blockers.append('Launch policy GPU selection is not confirmed')
    try:
        address = str(ipaddress.IPv4Address(streaming.get('public_ip')))
    except (ipaddress.AddressValueError, TypeError):
        address = None
        blockers.append('Streaming IPv4 address is not selected')
    if streaming.get('endpoint_verified') is not True:
        blockers.append('Streaming endpoint and client settings have not been verified')
    ports = [streaming.get(k) for k in ['signaling_port', 'media_port', 'http_port']]
    if any(type(p) is not int or not 1024 <= p <= 65535 for p in ports) or len(set(ports)) != 3:
        raise ValueError('Streaming ports must be three distinct unprivileged ports')
    if config['preflight'].get('thresholds_are_provisional') is not False:
        blockers.append('Resource thresholds are provisional and not released for execution')
    limits = config['preflight']
    ranges = {'samples': (3, 10), 'interval_seconds': (1, 10),
              'maximum_gpu_utilization_percent': (0, 100),
              'maximum_gpu_temperature_celsius': (1, 85),
              'minimum_free_vram_mib': (1, 49140),
              'minimum_disk_free_bytes': (21474836480, 10**15)}
    for name, (low, high) in ranges.items():
        if type(limits.get(name)) is not int or not low <= limits[name] <= high:
            raise ValueError(f'Invalid preflight limit: {name}')
    args = list(expected_args)
    args += [
        f'--/omni/replicator/backends/disk/root_dir={root}/outputs/replicator',
        f'--/app/livestream/outDirectory={root}/outputs/streaming',
        f'--/app/livestream/port={ports[0]}',
        f'--/app/livestream/fixedHostPort={ports[1]}',
        f'--/app/livestream/minHostPort={ports[1]}',
        f'--/app/livestream/maxHostPort={ports[1]}',
        f'--/exts/omni.services.transport.server.http/port={ports[2]}',
    ]
    if address:
        args.append(f'--/app/livestream/publicEndpointAddress={address}')
    if type(gpu.get('vulkan_index')) is int:
        args.append(f'--/renderer/activeGpu={gpu["vulkan_index"]}')
    if type(gpu.get('cuda_index')) is int:
        args.append(f'--/physics/cudaDevice={gpu["cuda_index"]}')
    runtime_env = {'PATH': '/usr/bin:/bin', 'LANG': 'C.UTF-8', 'LC_ALL': 'C.UTF-8',
                   'OMP_NUM_THREADS': '4', 'OPENBLAS_NUM_THREADS': '4',
                   'RESOURCE_NAME': 'IsaacSim', 'OLD_PYTHONPATH': ''}
    runtime_env.update(env)
    return {
        'schema_version': 1, 'status': 'prepared_not_executed',
        'root': str(root), 'cwd': str(cwd), 'launch_blockers': blockers,
        'simulation_started': False, 'runtime_verified': False,
        'environment': runtime_env,
        'command': ['/usr/bin/nice', '-n', '15', '/usr/bin/ionice', '-c', '3',
                    str(root/'issacsim/kit/kit'),
                    str(root/'issacsim/apps/isaacsim.exp.full.streaming.kit'), *args],
        'gpu': gpu, 'streaming': streaming, 'preflight': config['preflight'],
        'recording': {'automatic_start': False, 'output_root': str(root/'outputs'),
                      'cwd_default_synthetic_recorder': str(cwd/'_out_sdrec'),
                      'default_relative_replicator_root': str(root/'outputs/replicator')},
        'limits': {'filesystem_sandbox': False, 'builtin_movie_and_stage_output_override': False},
    }


def _command(args):
    return subprocess.run(args, check=True, capture_output=True, text=True, timeout=15,
                          env={'PATH': '/usr/bin:/bin', 'LANG': 'C', 'LC_ALL': 'C'}).stdout


def gpu_snapshot() -> dict:
    output = _command(['nvidia-smi', '--query-gpu=index,uuid,utilization.gpu,memory.free,temperature.gpu',
                       '--format=csv,noheader,nounits'])
    devices = {}
    for row in csv.reader(output.splitlines(), skipinitialspace=True):
        index, uid, use, free, temp = row
        devices[int(index)] = {'uuid': uid, 'utilization': int(use),
                               'free_mib': int(free), 'temperature': int(temp)}
    processes = []
    # The compute-only query omits graphics jobs, including another Isaac GUI.
    tree = ET.fromstring(_command(['nvidia-smi', '-q', '-x']))
    for gpu in tree.findall('gpu'):
        uid = gpu.findtext('uuid')
        for process in gpu.findall('processes/process_info'):
            processes.append({'gpu_uuid': uid, 'pid': int(process.findtext('pid')),
                              'type': process.findtext('type'),
                              'name': process.findtext('process_name')})
    return {'devices': devices, 'processes': processes}


def _port_conflicts(ports):
    output = _command(['ss', '-H', '-lntu'])
    conflicts = []
    for line in output.splitlines():
        columns = line.split()
        if len(columns) >= 5:
            suffix = columns[4].rsplit(':', 1)[-1]
            if suffix.isdigit() and int(suffix) in ports:
                conflicts.append(line)
    return conflicts


def _top_header():
    output = _command(['top', '-b', '-d', '1', '-n', '2']).splitlines()
    starts = [n for n, line in enumerate(output) if line.startswith('top -')]
    if not starts:
        raise RuntimeError('Could not inspect CPU with top')
    header = output[starts[-1]:starts[-1]+5]
    match = re.search(r'([0-9.]+)\s+id', '\n'.join(header))
    if not match:
        raise RuntimeError('Could not read CPU idle percentage')
    return {'idle_percent': float(match.group(1)), 'top_header': header}


def inspect_resources(plan):
    """Read-only preflight. Existing compute/graphics jobs prevent launch."""
    index = plan['gpu']['nvidia_index']
    policy = plan['preflight']
    errors, samples = [], []
    for _ in range(int(policy['samples'])):
        snapshot = gpu_snapshot()
        device = snapshot['devices'].get(index)
        if device is None:
            raise RuntimeError('Selected NVIDIA GPU does not exist')
        existing = [p for p in snapshot['processes'] if p['gpu_uuid'] == device['uuid']]
        if existing:
            pids = ','.join(str(p['pid']) for p in existing)
            owners = _command(['ps', '-o', 'user,pid,comm', '-p', pids])
            errors.append(f'Selected GPU has existing compute/graphics jobs; do not stop them:\n{owners}')
        if device['utilization'] > policy['maximum_gpu_utilization_percent']:
            errors.append('GPU compute utilization exceeds preflight policy')
        if device['free_mib'] < policy['minimum_free_vram_mib']:
            errors.append('Free VRAM is below preflight reserve')
        if device['temperature'] > policy['maximum_gpu_temperature_celsius']:
            errors.append('GPU temperature exceeds provisional preflight policy')
        samples.append(snapshot)
        if len(samples) < policy['samples']:
            time.sleep(policy['interval_seconds'])
    cpu = _top_header()
    if cpu['idle_percent'] < 50:
        errors.append('CPU idle is below initial 50 percent preflight policy')
    ports = [plan['streaming'][k] for k in ['signaling_port', 'media_port', 'http_port']]
    if _port_conflicts(ports):
        errors.append('A streaming port is already in use; existing listeners were not changed')
    if shutil.disk_usage(plan['root']).free < policy['minimum_disk_free_bytes']:
        errors.append('Filesystem free space is below reserved floor')
    return {'samples': samples, 'cpu': cpu, 'errors': sorted(set(errors))}


def _start_ticks(pid):
    text = Path(f'/proc/{pid}/stat').read_text()
    return int(text[text.rfind(')')+2:].split()[19])


def _stop_our_child(child):
    """Popen child only; never pgrep/pkill, process groups, or existing jobs."""
    if child.poll() is None:
        child.terminate()
        try:
            child.wait(timeout=20)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=10)


def execute(root) -> int:
    plan = build_plan(root)
    if plan['launch_blockers']:
        raise RuntimeError('Launch refused: ' + '; '.join(plan['launch_blockers']))
    if os.name != 'posix' or Path(plan['root']) != SERVER_ROOT:
        raise RuntimeError('Execution is restricted to the reviewed server workspace')
    for executable in ['/usr/bin/nice', '/usr/bin/ionice']:
        if not Path(executable).is_file():
            raise RuntimeError(f'Required executable missing: {executable}')
    resources = inspect_resources(plan)
    if resources['errors']:
        raise RuntimeError('Preflight refused: ' + '; '.join(resources['errors']))
    # Re-read immediately before writes/start to catch changes since preflight.
    if build_plan(root) != plan:
        raise RuntimeError('Configuration changed during preflight')
    root = Path(root)
    launch_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '_' + uuid.uuid4().hex[:12]
    run = confined_path(root, f'runtime/run/{launch_id}')
    ensure_directory(root, run.parent)
    run.mkdir(mode=0o700, exist_ok=False)
    for directory in [plan['cwd'], root/'outputs/replicator', root/'outputs/streaming']:
        ensure_directory(root, directory)
    token = uuid.uuid4().hex
    env = dict(plan['environment'])
    env.update(MRO_LAUNCH_ID=launch_id, MRO_LAUNCH_TOKEN=token)
    # Per-launch Kit and supervisor logs avoid reusing previous logs.
    command = [arg.replace(f'--/log/file={root}/runtime/logs/isaac-sim.log',
                           f'--/log/file={root}/runtime/logs/{launch_id}.kit.log')
               for arg in plan['command']]
    log = confined_path(root, f'runtime/logs/{launch_id}.console.log')
    ensure_directory(root, log.parent)
    atomic_replace_json(root, run/'preflight.json', resources)
    child = None
    owner = {'root': str(root), 'launch_id': launch_id, 'token': token, 'status': 'starting',
             'started_at': datetime.now(timezone.utc).isoformat(), 'pid': None}
    atomic_replace_json(root, run/'owner.json', owner)
    reason = 'child exited'
    try:
        with open(log, 'xb', buffering=0) as stream:
            child = subprocess.Popen(command, cwd=plan['cwd'], env=env, stdout=stream,
                                     stderr=subprocess.STDOUT, start_new_session=True)
            owner.update(pid=child.pid, process_start_ticks=_start_ticks(child.pid), status='running')
            atomic_replace_json(root, run/'owner.json', owner)
            deadline = time.monotonic() + 120
            while child.poll() is None:
                time.sleep(5)
                if child.poll() is not None:
                    break
                if read_json(root, 'config/launch_policy.json').get('simulation_launch_allowed') is not True:
                    reason = 'launch pause restored'; break
                snapshot = gpu_snapshot()
                device = snapshot['devices'][plan['gpu']['nvidia_index']]
                competing = [p for p in snapshot['processes']
                             if p['gpu_uuid'] == device['uuid'] and p['pid'] != child.pid]
                cpu = _top_header()
                atomic_replace_json(root, run/'latest_resources.json', {'gpu': snapshot, 'cpu': cpu})
                if competing:
                    reason = 'another GPU process appeared; stopping only our child'; break
                if device['free_mib'] < 8192 or device['temperature'] > 82:
                    reason = 'runtime VRAM/temperature reserve reached'; break
                if cpu['idle_percent'] < 10 or shutil.disk_usage(root).free < plan['preflight']['minimum_disk_free_bytes']:
                    reason = 'CPU/disk reserve reached'; break
                if time.monotonic() >= deadline:
                    reason = 'initial supervised session reached 120 second cap'; break
    except (KeyboardInterrupt, Exception) as exc:
        reason = f'supervisor stopped: {type(exc).__name__}: {exc}'
    finally:
        if child is not None:
            _stop_our_child(child)
        owner.update(status='stopped', stopped_at=datetime.now(timezone.utc).isoformat(),
                     stop_reason=reason, returncode=None if child is None else child.returncode)
        atomic_replace_json(root, run/'owner.json', owner)
    print(json.dumps({'launch_id': launch_id, 'status': owner['status'], 'reason': reason,
                      'returncode': owner['returncode']}, ensure_ascii=False, indent=2))
    return 0 if owner['returncode'] == 0 and reason == 'child exited' else 1


def capture_configuration_summary(root):
    config = read_json(root, 'config/recording.json')
    models = read_json(root, 'config/camera_models.json')['models']
    layout = read_json(root, 'config/camera_layout.json')
    instances = {entry['id']: entry for entry in layout['instances']}
    if len(instances) != len(layout['instances']):
        raise ValueError('Duplicate camera identifier')
    if config['output_root'] != 'outputs' or config['session_unique'] is not True:
        raise ValueError('Recording must use unique sessions below outputs')
    for item in instances.values():
        model = models[item['model']]
        for key in ['native_resolution_px', 'preview_resolution_px']:
            size = model[key]
            if len(size) != 2 or any(type(v) is not int or v < 1 for v in size):
                raise ValueError('Invalid camera resolution')
    profiles = {}
    for name, profile in config['profiles'].items():
        if not profile['camera_ids'] or any(cam not in instances for cam in profile['camera_ids']):
            raise ValueError('Profile references an unknown camera')
        limits = profile['limits']
        if any(type(limits[key]) is not int or limits[key] <= 0
               for key in ['max_frames', 'max_bytes', 'min_free_bytes']):
            raise ValueError('Recording limits must be positive integers')
        profiles[name] = {'enabled': profile['enabled'], 'camera_ids': profile['camera_ids'],
                          'max_frames': limits['max_frames'], 'bindings_verified': profile['bindings_verified']}
    return {'enabled': config['enabled'], 'fps_requested': config['fps'],
            'layout_enabled': layout['enabled'], 'profiles': profiles,
            'independent_odometry_connected': config['odometry']['provider_connected'],
            'runtime_synchronization_verified': config['synchronization']['server_runtime_verified']}


def main(check_only=False) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    if not check_only:
        parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    try:
        if not check_only and args.execute:
            return execute(args.root)
        plan = build_plan(args.root)
        plan['capture_preparation'] = capture_configuration_summary(args.root)
        # Paths/settings only; no tokens/passwords or inherited environment dump.
        print(json.dumps(plan, ensure_ascii=False, indent=2))
        return 0
    except (ValueError, OSError, RuntimeError, KeyError) as exc:
        print(f'REFUSED: {exc}', file=sys.stderr)
        return 2
