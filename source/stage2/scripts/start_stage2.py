"""Start the reviewed GPU1 workbench. No simulation launch without --execute."""
import argparse
import datetime
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path('/mnt/DATA/workspace/ws_minho/mro_1')
PROJECT = ROOT / 'stage2'
sys.path[:0] = [str(PROJECT / 'scripts'), str(ROOT), str(ROOT / 'scripts')]
from mro_runtime.paths import (read_json, atomic_replace_json, ensure_private_file,
                               write_new_bytes)
from stage2_supervisor import load_plan, proc_stat, still_live
from mro_runtime.launcher import gpu_snapshot, _top_header


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    print(json.dumps({'gpu': gpu_snapshot(), 'cpu': _top_header()}, indent=2))
    if not args.execute:
        print('Read-only resource snapshot. Add --execute to start the separate stage2 GPU1 workbench.')
        return
    if PROJECT.is_symlink() or PROJECT.resolve(strict=True) != PROJECT:
        raise ValueError('Unexpected stage2 workspace resolution')
    lock_rel = 'stage2/runtime/run/workbench_start.lock'
    if not (ROOT / lock_rel).exists():
        try:
            write_new_bytes(ROOT, lock_rel, b'')
        except FileExistsError:
            pass
    encode_lock_rel = 'stage2/runtime/run/video_encode.lock'
    if not (ROOT / encode_lock_rel).exists():
        try: write_new_bytes(ROOT, encode_lock_rel, b'')
        except FileExistsError: pass
    # Read-only handles on existing shared coordination files: flock does
    # not modify their contents and prevents a concurrent original launch.
    with ensure_private_file(ROOT, lock_rel).open('r+') as lock, \
            ensure_private_file(ROOT, encode_lock_rel).open('r+') as encode_lock, \
            ensure_private_file(ROOT, 'runtime/run/workbench_start.lock').open('r') as original_lock, \
            ensure_private_file(ROOT, 'runtime/run/video_encode.lock').open('r') as original_encode_lock:
        for held in (original_lock, original_encode_lock, lock, encode_lock):
            fcntl.flock(held, fcntl.LOCK_EX | fcntl.LOCK_NB)
        original_manifest = read_json(ROOT, 'manifests/current_view_supervisor.json')
        if still_live(original_manifest['identity']):
            raise RuntimeError('Original owned supervisor is still running; stop only its reviewed owner first')
        original_cfg = read_json(ROOT, 'config/viewing_session.json')
        original_launch = original_cfg.get('consumed_by_launch_id')
        if not original_launch:
            raise RuntimeError('Original launch identity is unavailable')
        original_owner = read_json(ROOT, 'runtime/run/' + original_launch + '/owner.json')
        if original_owner.get('status') != 'stopped' or original_owner.get('remaining_owned_processes') or original_owner.get('cleanup_errors'):
            raise RuntimeError('Original owned session cleanup is not confirmed')
        manifest = ROOT / 'stage2/manifests/current_view_supervisor.json'
        if manifest.exists():
            previous = read_json(ROOT, manifest)
            if still_live(previous['identity']):
                raise RuntimeError('An owned supervisor is already active; use mro_workbench.py status')
        stage1_cfg=read_json(ROOT,'stage1/config/viewing_session.json')
        stage1_owner=read_json(ROOT,'stage1/runtime/run/'+stage1_cfg['consumed_by_launch_id']+'/owner.json')
        assert stage1_owner['status']=='stopped' and not stage1_owner['remaining_owned_processes'] and not stage1_owner['cleanup_errors']
        cfg = read_json(ROOT, 'stage2/config/viewing_session.json')
        launch_id = cfg.get('consumed_by_launch_id')
        if launch_id:
            owner = read_json(ROOT, 'stage2/runtime/run/' + launch_id + '/owner.json')
            if owner.get('status') != 'stopped' or owner.get('cleanup_errors'):
                raise RuntimeError('Previous session cleanup is not confirmed; inspect its owner.json')
            if owner.get('remaining_owned_processes'):
                prior=owner['remaining_owned_processes']
                if any(still_live(row) for row in prior):
                    raise RuntimeError('Previous positively owned descendants remain alive')
                atomic_replace_json(ROOT, 'stage2/runtime/run/' + launch_id + '/cleanup_reverification.json',
                    {'status':'prior_remaining_identities_no_longer_live','identities':prior,
                     'original_owner_report_preserved':True,'checked_at':datetime.datetime.now(datetime.timezone.utc).isoformat()})
        # The launch gate, GPU mapping and all path checks remain authoritative.
        cfg.update(enabled=True, stop_requested=False)
        atomic_replace_json(ROOT, 'stage2/config/viewing_session.json', cfg)
        child = None
        try:
            load_plan()
            stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
            log_rel = 'stage2/runtime/logs/view_supervisor_' + stamp + '.log'
            write_new_bytes(ROOT, log_rel, b'')
            env = {'PATH': '/usr/bin:/bin', 'HOME': str(PROJECT / 'runtime/home'),
                   'TMPDIR': str(PROJECT / 'runtime/tmp'), 'PYTHONDONTWRITEBYTECODE': '1',
                   'LANG': 'C.UTF-8', 'LC_ALL': 'C.UTF-8'}
            with ensure_private_file(ROOT, log_rel).open('ab') as log:
                child = subprocess.Popen(['/usr/bin/python3', '-B',
                    str(PROJECT / 'scripts/stage2_supervisor.py'), '--execute'], cwd=ROOT,
                    env=env, stdin=subprocess.DEVNULL, stdout=log,
                    stderr=subprocess.STDOUT, start_new_session=True)
            identity = proc_stat(child.pid)
            if identity is None:
                raise RuntimeError('Supervisor exited immediately; inspect ' + log_rel)
            result = {'pid': child.pid, 'identity': identity,
                      'supervisor_log': str(ROOT / log_rel), 'started_at': stamp}
            atomic_replace_json(ROOT, manifest, result)
            print(json.dumps(result, indent=2))
            print('Starting: wait for stage_status=stage2_open before WebRTC Connect.')
        except Exception:
            # Terminate only the direct supervisor we just started. It owns and
            # cleans its own sandbox descendants; preserve its consumed state.
            if child is not None and child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    latest = read_json(ROOT, 'stage2/config/viewing_session.json')
                    latest.update(enabled=False, stop_requested=True)
                    atomic_replace_json(ROOT, 'stage2/config/viewing_session.json', latest)
                    raise RuntimeError('Owned supervisor cleanup is still pending; inspect its log')
            latest = read_json(ROOT, 'stage2/config/viewing_session.json')
            latest['enabled'] = False
            atomic_replace_json(ROOT, 'stage2/config/viewing_session.json', latest)
            raise


if __name__ == '__main__':
    main()
