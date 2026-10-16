"""Static public-tree checks. Does not train, step physics or open hardware."""
from pathlib import Path
import ast,json,hashlib,re,sys
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parent.parent
errors=[]
for p in ROOT.rglob('*'):
 if not p.is_file() or '.git' in p.parts:continue
 if p.suffix=='.py':
  try:ast.parse(p.read_text(),filename=str(p))
  except SyntaxError as e:errors.append(str(e))
 if p.name.endswith('_calibration.json') or p.suffix=='.pt':errors.append(f'private calibration/checkpoint included: {p}')
 if p.suffix in ('.py','.md','.json','.toml','.xml'):
  for match in re.findall(r'/home/(?:ubuntu|pi)/[^\s"<>`]+',p.read_text()):
   # Pi destination examples are intentionally user configurable.
   if not match.startswith('/home/pi/'):errors.append(f'personal path in {p}: {match}')
registry=ROOT/'software/training/src/mjlab_microduck/tasks/__init__.py'
t=ast.parse(registry.read_text())
calls=[n for n in ast.walk(t) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='register_mjlab_task']
if len(calls)!=1:errors.append(f'expected one MicroDuck registration, got {len(calls)}')
manifest=ROOT/'software/deployment/teleop/scripts/models/microduck_s288_v0_backlash_dr_rotor_pd_m1.manifest.json'
m=json.loads(manifest.read_text());model=manifest.with_suffix('').with_suffix('.onnx')
if hashlib.sha256(model.read_bytes()).hexdigest()!=m['onnx_sha256']:errors.append('model sha256 mismatch')
# Check repository-relative Markdown links; placeholders and external links skipped.
for p in ROOT.rglob('*.md'):
 for target in re.findall(r'!?\[[^\]]*\]\(([^)]+)\)',p.read_text()):
  if '://' in target or target.startswith('#'):continue
  target=target.split('#')[0]
  if target and not (p.parent/target).exists():errors.append(f'broken link: {p.relative_to(ROOT)} -> {target}')
if errors:
 print('\n'.join(errors));sys.exit(1)
print('PASS: Python syntax, one task registration, no private calibration/PT, model hash, relative document links')
