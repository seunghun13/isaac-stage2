"""Read-only publication verification; no SSH, ROS, or simulator startup."""
from pathlib import Path
import ast
import hashlib
import json
import re
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    core = json.loads((ROOT/'provenance/CAPTURE_SOURCES.json').read_text(encoding='utf-8'))
    origins = json.loads((ROOT/'provenance/FILES.json').read_text(encoding='utf-8'))
    original_manifest = json.loads((ROOT/'source/stage2/integration_20260920_v7/SOURCE_MANIFEST.json').read_text(encoding='utf-8'))
    if len(core) != 29 or core != original_manifest:
        raise ValueError('Capture manifest does not match the preserved release')
    for rel, expected in core.items():
        if digest(ROOT/'source'/rel) != expected:
            raise ValueError('Capture source changed: '+rel)
    for rel, record in origins.items():
        p = ROOT/rel
        if p.stat().st_size != record['bytes'] or digest(p) != record['sha256']:
            raise ValueError('Published file changed: '+rel)
    counts = {'python':0,'json':0,'xml':0,'markdown_links':0}
    for p in ROOT.rglob('*'):
        if not p.is_file() or '.git' in p.relative_to(ROOT).parts:
            continue
        if p.suffix == '.py':
            ast.parse(p.read_bytes(), filename=p.relative_to(ROOT).as_posix())
            counts['python'] += 1
        elif p.suffix == '.json':
            json.loads(p.read_text(encoding='utf-8'))
            counts['json'] += 1
        elif p.suffix == '.xml':
            ET.parse(p)
            counts['xml'] += 1
        elif p.suffix == '.md':
            for target in re.findall(r'\]\(([^)]+)\)',p.read_text(encoding='utf-8')):
                if target.startswith(('http:','https:','#','mailto:')):
                    continue
                if not (p.parent/target.split('#')[0]).exists():
                    raise ValueError('Broken link: '+str(p.relative_to(ROOT))+' -> '+target)
                counts['markdown_links'] += 1
    evidence = json.loads((ROOT/'evidence/final_noqr/final01_audit01.json').read_text(encoding='utf-8'))['result']
    acceptance = json.loads((ROOT/'evidence/final_noqr/ACCEPTANCE.json').read_text(encoding='utf-8'))
    if not (evidence['status']=='pass' and evidence['agreed_scene_step_sync_pass']
            and evidence['qr_absence_verified'] and len(evidence['topic_counts'])==18
            and evidence['clock_steps']==1801 and evidence['image_source_hash_checks']==1353
            and evidence['camera_image_content_time_verified'] is None
            and acceptance['fine_global_beam_emission_verified'] is False
            and acceptance['all_sensor_acquisition_sync_verified'] is False):
        raise ValueError('Evidence scope no longer matches the documented final collection')
    print(json.dumps({'status':'pass','capture_sources':len(core),'published_origin_files':len(origins),
                      **counts,'scope':'file integrity, syntax and historical evidence consistency only',
                      'network_used':False,'simulator_started':False},ensure_ascii=False))


if __name__ == '__main__':
    main()
