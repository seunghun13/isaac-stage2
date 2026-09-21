"""One-use GPU 1 session with an optional deadline. Default: print a plan only.

Only the positively identified sandbox child and its descendants may be stopped.
The parent application prepares/validates GPU mappings and the bootstrap script.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

ROOT = Path('/mnt/DATA/workspace/ws_minho/mro_1')
PROJECT = ROOT / 'stage2'
GPU_UUID = 'GPU-f6c40441-5fcb-3a31-7115-c7c4c1be7d8b'
POLICY = 'stage2/config/viewing_session.json'
# Observed nvidia-smi GPU1 temperatures on 2026-09-10: target84C,
# max-operating93C, slowdown95C, shutdown98C. Allow up to85C during our
# workload (target+1, max-operating-8); retain the separate75C launch gate.
# This is our monitored operating policy, not guaranteed hardware protection;
# no fan, power-limit or server-wide thermal setting is changed.
GPU1_MAX_RUNTIME_TEMPERATURE_C = 85
sys.path[:0] = [str(PROJECT / 'scripts'), str(ROOT / 'scripts'), str(ROOT)]
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'mro1_runtime_prep'))
from mro_runtime.paths import (confined_path, ensure_directory, ensure_private_file,
                               read_json, atomic_replace_json)
from mro_runtime.launcher import (REQUIRED_PATH_ENV, ALLOWED_SCALAR_ENV,
                                  gpu_snapshot, _top_header, _port_conflicts)


def gate_released(gate):
    return (gate.get('simulation_launch_allowed') is True
            and gate.get('gpu_selection') == 1
            and gate.get('scope') in ('view_only', 'camera_validation'))


def session_deadline(config, now):
    # Only an explicit JSON null disables elapsed-time shutdown.
    if 'max_seconds' not in config:
        raise ValueError('max_seconds must be explicit: null or 30 to 900 seconds')
    duration = config['max_seconds']
    if duration is None:
        return None
    if type(duration) is not int or not 30 <= duration <= 900:
        raise ValueError('Session duration must be null or 30 to 900 seconds')
    return now + duration


def deadline_expired(deadline, now):
    return deadline is not None and now >= deadline


def gpu1_min_free_vram_mib(config):
    """Explicit MiB reserve, fixed for a launch; no implicit 24 GiB fallback."""
    value = config.get('gpu1_min_free_vram_mib')
    # GPU_UUID identifies the inspected A6000 with memory.total=49140 MiB.
    # This only validates units/range; it does not prove workload capacity.
    if type(value) is not int or not 0 < value < 49140:
        raise ValueError('gpu1_min_free_vram_mib must be an explicit integer between 1 and 49139 MiB')
    return value


def runtime_gpu_reserve_reached(gpu, min_free_mib):
    return gpu['free_mib'] < min_free_mib or gpu['temperature'] > GPU1_MAX_RUNTIME_TEMPERATURE_C


def proc_stat(pid):
    try:
        value = Path(f'/proc/{int(pid)}/stat').read_text()
        fields = value[value.rfind(')') + 2:].split()
        return {'pid': int(pid), 'ppid': int(fields[1]), 'start_ticks': int(fields[19]),
                'state': fields[0]}
    except (FileNotFoundError, ProcessLookupError):
        return None


def identity(pid):
    value = proc_stat(pid)
    if value is None:
        return None
    try:
        status = Path(f'/proc/{pid}/status').read_text()
        value['uid'] = int(next(s for s in status.splitlines() if s.startswith('Uid:')).split()[1])
        value['exe'] = os.readlink(f'/proc/{pid}/exe')
        # Reading several proc files is not atomic; reject PID replacement.
        check = proc_stat(pid)
        return value if check and check['start_ticks'] == value['start_ticks'] else None
    except (FileNotFoundError, ProcessLookupError):
        return None


def same_process(expected, actual):
    return bool(actual and all(actual.get(k) == expected.get(k)
                               for k in ('pid', 'start_ticks')))


def xorg_allowed(process, allowed, actual):
    return (process['gpu_uuid'] == GPU_UUID and process['type'] == 'G'
            and process['pid'] == allowed['pid'] and bool(actual)
            and all(actual.get(k) == allowed[k] for k in ('pid', 'start_ticks', 'uid', 'exe')))


def collect_owned(root_identity, known):
    """Host /proc ancestry, retaining proven descendants if they are reparented."""
    table = {}
    for item in Path('/proc').iterdir():
        if item.name.isdigit():
            try:
                row = proc_stat(int(item.name))
            except PermissionError:
                continue
            if row:
                table[row['pid']] = row
    owned = {pid: row for pid, row in known.items() if same_process(row, table.get(pid))}
    if same_process(root_identity, table.get(root_identity['pid'])):
        owned[root_identity['pid']] = root_identity
    changed = True
    while changed:
        changed = False
        for pid, row in table.items():
            if pid not in owned and row['ppid'] in owned:
                owned[pid] = row
                changed = True
    known.update(owned)
    return owned


def process_key(process, read_stat=proc_stat):
    row = read_stat(process['pid'])
    if row is None:
        raise RuntimeError('GPU process changed during inspection; retry a fresh preflight')
    return (process['gpu_uuid'], row['pid'], row['start_ticks'])


def classify(snapshot, owned, xorg, baseline_gpu0, read_identity=identity, read_stat=proc_stat,
             previously_owned=None):
    devices = snapshot['devices']
    if devices.get(1, {}).get('uuid') != GPU_UUID or 0 not in devices:
        return 'GPU identity or inventory changed'
    for process in snapshot['processes']:
        pid = process['pid']
        current = read_stat(pid)
        ours = pid in owned and same_process(owned[pid], current)
        if ours and process['gpu_uuid'] != GPU_UUID:
            return 'Our process appeared on another GPU'
        if process['gpu_uuid'] == GPU_UUID:
            prior = (previously_owned or {}).get(pid)
            if prior and (current is None or (same_process(prior, current) and current.get('state') == 'Z')):
                return 'GPU snapshot lists an exited PID previously identified as ours; stopping conservatively'
            if not ours and not xorg_allowed(process, xorg, read_identity(pid)):
                return 'Another GPU 1 process is present'
        elif 'C' in process['type'] and process_key(process, read_stat) not in baseline_gpu0:
            return 'A new compute process appeared on GPU 0'
    return None


def signal_owned(row, sig, read_stat=proc_stat, send=os.kill):
    if same_process(row, read_stat(row['pid'])):
        try:
            send(row['pid'], sig)
            return True
        except ProcessLookupError:
            pass
    return False


def still_live(row):
    actual = proc_stat(row['pid'])
    return same_process(row, actual) and actual.get('state') != 'Z'


def stop_direct_child(child):
    if child is not None and child.poll() is None:
        child.terminate()
        try:
            child.wait(timeout=10)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=10)


def stop_owned(child, root_identity, known):
    """Never signal by name, account, namespace-wide PID, or process group."""
    try:
        try:
            collect_owned(root_identity, known)
        except Exception as exc:
            # Still stop already proven descendants and the exact Popen child.
            print(f'Cleanup ancestry refresh failed: {type(exc).__name__}: {exc}', file=sys.stderr, flush=True)
        descendants = [row for pid, row in known.items() if pid != child.pid]
        for row in reversed(descendants):
            signal_owned(row, signal.SIGTERM)
        until = time.monotonic() + 10
        while time.monotonic() < until and any(still_live(row) for row in descendants):
            time.sleep(0.25)
        for row in reversed(descendants):
            signal_owned(row, signal.SIGKILL)
    finally:
        # Popen's unreaped direct child remains positively owned even if /proc fails.
        stop_direct_child(child)
    return [row for row in known.values() if still_live(row)]


def load_plan():
    root = ROOT.resolve(strict=True)
    if root != ROOT or ROOT.is_symlink():
        raise ValueError('Unexpected workspace resolution')
    if PROJECT.is_symlink() or PROJECT.resolve(strict=True) != PROJECT:
        raise ValueError('Unexpected stage2 workspace resolution')
    cfg = read_json(ROOT, POLICY)
    gate = read_json(ROOT, 'stage2/config/launch_policy.json')
    if not (cfg.get('enabled') is True and gate_released(gate)):
        raise ValueError('The one-use GPU 1 viewing policy is not released')
    gpu = cfg['gpu']
    if not (gpu.get('nvidia_index') == 1 and gpu.get('uuid') == GPU_UUID
            and gpu.get('mapping_verified') is True
            and type(gpu.get('vulkan_index')) is int and gpu['vulkan_index'] >= 0
            and gpu.get('cuda_index') == 0):
        raise ValueError('GPU mapping has not been verified for this sandbox')
    session_deadline(cfg, 0)
    gpu1_min_free_vram_mib(cfg)
    xorg = cfg['xorg']
    if not (type(xorg.get('pid')) is int and type(xorg.get('start_ticks')) is int
            and type(xorg.get('uid')) is int and xorg.get('exe') == '/usr/lib/xorg/Xorg'):
        raise ValueError('Exact pre-existing Xorg identity is required')
    streaming = cfg['streaming']
    expected = {'public_ip': '192.0.2.1', 'signaling_port': 49100,
                'media_port': 47998, 'http_port': 8011}
    if any(streaming.get(k) != v for k, v in expected.items()):
        raise ValueError('Unreviewed streaming endpoint')
    bootstrap = ensure_private_file(ROOT, cfg['bootstrap'])
    if bootstrap != PROJECT / 'scripts/view_hangar_bootstrap.py':
        raise ValueError('Unreviewed bootstrap script')
    env = read_json(ROOT, 'stage2/config/runtime_environment.json')
    if set(env) != REQUIRED_PATH_ENV | ALLOWED_SCALAR_ENV:
        raise ValueError('Unexpected environment keys')
    for key in REQUIRED_PATH_ENV:
        if key == 'MRO_ROOT':
            if env[key] != str(ROOT):
                raise ValueError('Unexpected read-only source root')
        elif key == 'ISAAC_PATH':
            if env[key] != str(ROOT / 'issacsim'):
                raise ValueError('Unexpected read-only Isaac installation')
            confined_path(ROOT, env[key], require_exists=True)
        else:
            confined_path(PROJECT, env[key], require_exists=True)
        if not Path(env[key]).is_absolute():
            raise ValueError('Runtime paths must be absolute')
    if env['HOME'] != str(PROJECT / 'runtime/home') or env['MRO_ROOT'] != str(ROOT):
        raise ValueError('Unexpected HOME or MRO_ROOT')
    if any(env[k] != '1' for k in ALLOWED_SCALAR_ENV):
        raise ValueError('Python isolation flags are required')
    omni = ensure_private_file(ROOT, 'stage2/runtime/config/omniverse/omniverse.toml')
    lines = [s.strip() for s in omni.read_text().splitlines() if s.strip() and not s.lstrip().startswith('#')]
    if lines != ['[paths]'] + [f'{k}_root = "{PROJECT}/runtime/{k}"' for k in ('data', 'cache', 'logs')]:
        raise ValueError('Omniverse write paths changed')
    ensure_private_file(ROOT, 'stage2/config/user.config.json')
    env.update(PATH='/usr/bin:/bin', LANG='C.UTF-8', LC_ALL='C.UTF-8',
               OMP_NUM_THREADS='4', OPENBLAS_NUM_THREADS='4', RESOURCE_NAME='IsaacSim',
               OLD_PYTHONPATH='', CUDA_VISIBLE_DEVICES=GPU_UUID,
               OPTIX_CACHE_PATH=str(confined_path(ROOT, 'stage2/runtime/cache/optix')),
               VK_ICD_FILENAMES='/usr/share/vulkan/icd.d/nvidia_icd.json')
    kit = ensure_private_file(ROOT, 'issacsim/kit/kit')
    app = ensure_private_file(ROOT, 'issacsim/apps/isaacsim.exp.full.streaming.kit')
    args = [str(kit), str(app), '--no-window', '--no-ros-env', '--portable-root',
            str(PROJECT / 'runtime/portable'), f'--/app/userConfigPath={PROJECT}/config/user.config.json',
            '--/renderer/multiGpu/enabled=false', '--/renderer/multiGpu/autoEnable=false',
            '--/renderer/multiGpu/maxGpuCount=1', f'--/renderer/activeGpu={gpu["vulkan_index"]}',
            '--/physics/cudaDevice=0', '--/isaac/startup/ros_bridge_extension=',
            '--/isaac/startup/create_new_stage=false',
            '--/app/window/width=1920', '--/app/window/height=1080',
            '--/app/renderer/resolution/width=1920', '--/app/renderer/resolution/height=1080',
            '--/app/livestream/allowResize=false', '--/app/livestream/nvcf/quitOnSessionEnded=false',
            '--/app/player/useFixedTimeStepping=true', '--/app/runLoops/main/manualModeEnabled=true',
            '--/app/runLoops/main/rateLimitEnabled=true', '--/app/runLoops/main/rateLimitFrequency=10',
            '--/app/runLoops/rendering_0/rateLimitEnabled=true',
            '--/app/runLoops/rendering_0/rateLimitFrequency=10',
            '--/app/livestream/publicEndpointAddress=192.0.2.1', '--/app/livestream/port=49100',
            '--/app/livestream/fixedHostPort=47998', '--/app/livestream/minHostPort=47998',
            '--/app/livestream/maxHostPort=47998', '--/exts/omni.services.transport.server.http/port=8011',
            f'--/omni/replicator/backends/disk/root_dir={PROJECT}/outputs/replicator',
            f'--/app/livestream/outDirectory={PROJECT}/outputs/streaming', '--exec', str(bootstrap)]
    sandbox = ['/usr/bin/bwrap', '--ro-bind', '/', '/', '--ro-bind', str(ROOT), str(ROOT),
               '--ro-bind', str(ROOT / 'hangar'), str(ROOT / 'hangar'),
               '--ro-bind', str(ROOT / 'robots'), str(ROOT / 'robots'), '--proc', '/proc', '--dev', '/dev']
    # Original project/installation/assets stay read-only. The stage2
    # accounting wrapper counts this complete new writable tree in addition
    # to the original campaign, retaining the original byte baseline.
    directory = ensure_directory(ROOT, 'stage2')
    sandbox += ['--bind', str(directory), str(directory)]
    for device in ('nvidia1', 'nvidiactl', 'nvidia-uvm', 'nvidia-uvm-tools', 'nvidia-modeset'):
        path = '/dev/' + device
        if not Path(path).exists():
            raise ValueError(f'Required GPU device is absent: {path}')
        sandbox += ['--dev-bind', path, path]
    sandbox += ['--dir', '/dev/dri']
    for device in ('card2', 'renderD129'):
        path = '/dev/dri/' + device
        if not Path(path).exists():
            raise ValueError(f'Required DRM device is absent: {path}')
        sandbox += ['--dev-bind', path, path]
    sandbox += ['--unshare-pid', '--die-with-parent', '--new-session', '--']
    return {'config': cfg, 'environment': env, 'command': sandbox + ['/usr/bin/nice', '-n', '15',
            '/usr/bin/ionice', '-c', '3'] + args, 'cwd': str(PROJECT / 'outputs/manual')}


def preflight(plan):
    import shutil
    from stage2_storage_budget import check_storage_budget
    min_free_mib = gpu1_min_free_vram_mib(plan['config'])
    storage = check_storage_budget(ROOT, incoming_bytes=256 * 1024**2)
    baseline = None
    for index in range(3):
        snapshot = gpu_snapshot()
        keys = {process_key(p) for p in snapshot['processes']
                if p['gpu_uuid'] != GPU_UUID and 'C' in p['type']}
        if baseline is None:
            baseline = keys
        reason = classify(snapshot, {}, plan['config']['xorg'], baseline)
        gpu = snapshot['devices'].get(1, {})
        if reason or gpu.get('utilization', 101) > 10 or gpu.get('temperature', 100) > 75 \
                or gpu.get('free_mib', 0) < min_free_mib or snapshot['devices'][0]['utilization'] > 20:
            raise RuntimeError(reason or 'Initial GPU resource limits were not met')
        if index < 2:
            time.sleep(2)
    cpu = _top_header()
    if cpu['idle_percent'] < 50 or shutil.disk_usage(ROOT).free < 50 * 1024**3:
        raise RuntimeError('Initial CPU or disk reserve was not met')
    if _port_conflicts([49100, 47998, 8011]):
        raise RuntimeError('A streaming port is already occupied')
    return {'gpu': snapshot, 'cpu': cpu, 'gpu0_baseline': sorted(baseline), 'storage_budget': storage,
            'gpu1_min_free_vram_mib': min_free_mib}


class StopSession(Exception):
    pass


def execute(plan):
    import shutil
    from stage2_storage_budget import check_storage_budget
    resources = preflight(plan)
    if load_plan() != plan:
        raise RuntimeError('Configuration changed during preflight')
    launch_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_') + uuid.uuid4().hex[:10]
    run = confined_path(ROOT, f'stage2/runtime/run/{launch_id}')
    ensure_directory(ROOT, run.parent)
    run.mkdir(mode=0o700)
    for path in ('stage2/runtime/logs', 'stage2/runtime/cache/optix', 'stage2/outputs/manual', 'stage2/outputs/replicator', 'stage2/outputs/streaming'):
        ensure_directory(ROOT, path)
    console = confined_path(ROOT, f'stage2/runtime/logs/{launch_id}.console.log')
    kitlog = confined_path(ROOT, f'stage2/runtime/logs/{launch_id}.kit.log')
    cfg = dict(plan['config'], enabled=False, consumed_by_launch_id=launch_id)
    # Runtime uses the launch snapshot, not subsequent edits to viewing_session.json.
    min_free_mib = gpu1_min_free_vram_mib(cfg)
    atomic_replace_json(ROOT, POLICY, cfg)
    atomic_replace_json(ROOT, run / 'preflight.json', resources)
    atomic_replace_json(ROOT, run / 'configuration.json', plan['config'])
    env = dict(plan['environment'], MRO_LAUNCH_ID=launch_id, MRO_VIEW_RUN=str(run), MRO_VIEW_RUN_DIR=str(run),
               MRO_LAUNCH_TOKEN=uuid.uuid4().hex)
    command = plan['command'] + [f'--/log/file={kitlog}']
    owner = {'launch_id': launch_id, 'supervisor_pid': os.getpid(), 'sandbox_pid': None,
             'status': 'starting', 'started_at': datetime.now(timezone.utc).isoformat(),
             'console_log': str(console), 'kit_log': str(kitlog), 'run_directory': str(run),
             'gpu1_min_free_vram_mib': min_free_mib}
    atomic_replace_json(ROOT, run / 'owner.json', owner)
    def stop_signal(number, frame):
        raise StopSession(f'Received signal {number}')
    handlers = {sig: signal.signal(sig, stop_signal) for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT)}
    child, root_identity, known, reason = None, None, {}, 'Child exited'
    try:
        with open(console, 'xb', buffering=0) as log, open(run / 'resources.jsonl', 'x', encoding='utf-8') as metrics:
            child = subprocess.Popen(command, cwd=plan['cwd'], env=env, stdout=log,
                                     stderr=subprocess.STDOUT, start_new_session=True)
            root_identity = proc_stat(child.pid)
            if root_identity is None:
                raise RuntimeError('Sandbox exited before its identity could be recorded')
            known[child.pid] = root_identity
            owner.update(sandbox_pid=child.pid, sandbox_start_ticks=root_identity['start_ticks'], status='running')
            atomic_replace_json(ROOT, run / 'owner.json', owner)
            print(json.dumps(owner), flush=True)
            next_sample = time.monotonic()
            deadline = session_deadline(cfg, next_sample)
            count = 0
            baseline = {tuple(row) for row in resources['gpu0_baseline']}
            while child.poll() is None:
                time.sleep(max(0, next_sample - time.monotonic()))
                next_sample = time.monotonic() + 3
                view = read_json(ROOT, POLICY)
                gate = read_json(ROOT, 'stage2/config/launch_policy.json')
                if (view.get('stop_requested') is True or view.get('consumed_by_launch_id') != launch_id
                        or not gate_released(gate)):
                    raise StopSession('Viewing policy requested a stop')
                snapshot, cpu = gpu_snapshot(), _top_header()
                owned = collect_owned(root_identity, known)
                problem = classify(snapshot, owned, cfg['xorg'], baseline, previously_owned=known)
                gpu = snapshot['devices'][1]
                storage = check_storage_budget(ROOT, incoming_bytes=256 * 1024**2)
                record = {'at': datetime.now(timezone.utc).isoformat(), 'gpu': snapshot, 'cpu': cpu,
                          'storage_budget': storage,
                          'gpu1_min_free_vram_mib': min_free_mib,
                          'owned_processes': list(owned.values()), 'classification': problem}
                atomic_replace_json(ROOT, run / 'latest_resources.json', record)
                if count < 400:
                    metrics.write(json.dumps(record) + '\n')
                    metrics.flush()
                    count += 1
                if problem:
                    raise StopSession(problem)
                if runtime_gpu_reserve_reached(gpu, min_free_mib):
                    raise StopSession(f'GPU 1 VRAM or temperature reserve reached '
                                      f'(free {gpu["free_mib"]} MiB, minimum {min_free_mib} MiB; '
                                      f'temperature {gpu["temperature"]} C, '
                                      f'maximum {GPU1_MAX_RUNTIME_TEMPERATURE_C} C)')
                if shutil.disk_usage(ROOT).free < 50 * 1024**3:
                    raise StopSession('Disk reserve reached')
                if deadline_expired(deadline, time.monotonic()):
                    raise StopSession('Bounded viewing session expired')
    except Exception as exc:
        reason = f'{type(exc).__name__}: {exc}'
    finally:
        # Further normal termination signals must not interrupt owned-child cleanup.
        for sig in handlers:
            signal.signal(sig, signal.SIG_IGN)
        remaining, cleanup_errors = [], []
        try:
            if child and root_identity:
                remaining = stop_owned(child, root_identity, known)
        except Exception as exc:
            cleanup_errors.append(f'{type(exc).__name__}: {exc}')
        finally:
            try:
                stop_direct_child(child)
            except Exception as exc:
                cleanup_errors.append(f'{type(exc).__name__}: {exc}')
        owner.update(status='stopped', stopped_at=datetime.now(timezone.utc).isoformat(), stop_reason=reason,
                     returncode=None if child is None else child.poll(), remaining_owned_processes=remaining,
                     cleanup_errors=cleanup_errors)
        atomic_replace_json(ROOT, run / 'owner.json', owner)
        print(json.dumps(owner), flush=True)
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    return 0 if reason == 'Child exited' and owner['returncode'] == 0 else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    try:
        plan = load_plan()
        if not args.execute:
            print(json.dumps(plan, indent=2))
            return 0
        return execute(plan)
    except Exception as exc:
        print(f'REFUSED: {type(exc).__name__}: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
