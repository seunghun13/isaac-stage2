"""Enforce the user's campaign growth cap and shared-volume free-space floor."""
import json
import math
import os
from pathlib import Path
import shutil
import stat
import sys
import time


class StorageBudgetExceeded(RuntimeError):
    pass


def tracked_bytes(root, directories):
    root = Path(root).resolve(strict=True)
    total, count, seen = 0, 0, set()
    for name in directories:
        rel = Path(name)
        if rel.is_absolute() or len(rel.parts) != 1 or name in ('.', '..'):
            raise ValueError('Storage budget directories must be direct owned children')
        base = root / rel
        if not base.exists():
            continue
        if base.is_symlink() or not base.resolve(strict=True).is_relative_to(root):
            raise ValueError('Storage accounting root is linked or outside the workspace')
        pending = [base]
        while pending:
            directory = pending.pop()
            with os.scandir(directory) as entries:
                for entry in entries:
                    try:
                        info = entry.stat(follow_symlinks=False)
                    except FileNotFoundError:
                        continue  # Atomic status-file replacement during accounting.
                    if stat.S_ISLNK(info.st_mode):
                        # Never traverse links while counting, and reject escapes.
                        if not Path(entry.path).resolve().is_relative_to(root):
                            raise ValueError('Storage accounting encountered an external symlink')
                    elif stat.S_ISDIR(info.st_mode):
                        pending.append(Path(entry.path))
                    elif stat.S_ISREG(info.st_mode):
                        identity = (info.st_dev, info.st_ino)
                        if identity not in seen:
                            total += info.st_size
                            count += 1
                            seen.add(identity)
    return total, count


def evaluate_budget(policy, current_bytes, disk_free_bytes, incoming_bytes=0):
    if type(incoming_bytes) is not int or incoming_bytes < 0:
        raise ValueError('incoming_bytes must be a nonnegative integer')
    floor, cap = policy['reserved_free_bytes'], policy['max_growth_bytes']
    baseline = policy['baseline_tracked_bytes']
    if any(type(x) is not int or x < 0 for x in [floor, cap, baseline, current_bytes, disk_free_bytes]):
        raise ValueError('Storage budget fields must be nonnegative integers')
    growth = max(0, current_bytes - baseline)
    campaign_remaining = max(0, cap - growth)
    available = max(0, min(campaign_remaining, disk_free_bytes - floor))
    result = {'current_tracked_bytes': current_bytes, 'baseline_tracked_bytes': baseline,
              'growth_bytes': growth, 'max_growth_bytes': cap,
              'campaign_remaining_bytes': campaign_remaining,
              'disk_free_bytes': disk_free_bytes, 'reserved_free_bytes': floor,
              'available_to_work_bytes': available, 'incoming_bytes': incoming_bytes}
    if growth + incoming_bytes > cap or disk_free_bytes - incoming_bytes < floor:
        raise StorageBudgetExceeded('User storage limit would be exceeded: ' + json.dumps(result))
    return result


def check_storage_budget(root, incoming_bytes=0):
    start = time.monotonic()
    root = Path(root).resolve(strict=True)
    sys.path.insert(0, str(root)) if str(root) not in sys.path else None
    from mro_runtime.paths import read_json
    policy = read_json(root, 'config/storage_budget.json')
    if policy.get('schema_version') != 1:
        raise ValueError('Unsupported storage budget policy')
    if policy['max_growth_bytes'] > 2 * 1024**4 or policy['reserved_free_bytes'] < math.ceil(1.13 * 1024**4):
        raise ValueError('Storage policy exceeds the user-authorized bounds')
    required = {'outputs', 'runtime', 'scripts', 'config', 'docs', 'manifests'}
    if set(policy.get('tracked_directories', [])) != required:
        raise ValueError('Storage accounting must include every mutable campaign directory')
    current, count = tracked_bytes(root, policy['tracked_directories'])
    result = evaluate_budget(policy, current, shutil.disk_usage(root).free, incoming_bytes)
    result.update(tracked_regular_files=count, accounting_wall_seconds=time.monotonic()-start)
    return result


if __name__ == '__main__':
    print(json.dumps(check_storage_budget('/mnt/DATA/workspace/ws_minho/mro_1'), indent=2))
