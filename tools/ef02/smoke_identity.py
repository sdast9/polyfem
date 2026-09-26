#!/usr/bin/env python3
"""Five option-off public smokes, sequential, comparing every exported VTU byte.

The exported solution/geometry files are the byte-identity oracle; provenance,
wall times, paths, and compiled source metadata in manifests necessarily differ.
"""
import argparse, hashlib, json, subprocess, sys
from pathlib import Path

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--workspace',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
p.add_argument('--production',type=Path,required=True);p.add_argument('--candidate',type=Path,required=True)
a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True);results=[]
for scene in ('smoke-qs','smoke-tr','smoke-adaptive','smoke-alhess','smoke-friction'):
    for kind,binary in [('base',a.production),('off',a.candidate)]:
        cmd=[sys.executable,str(Path(__file__).with_name('run.py')),'--workspace',str(a.workspace),
             f'{kind}-{scene}','--scene',scene,'--steps','4','--threads','1','--out',str(a.out),'--binary',str(binary)]
        if kind=='off':cmd+=['--set','/solver/contact/semi_implicit/band_statistic="rms"','--set','/solver/contact/semi_implicit/initial_trim_estimate=false']
        with (a.out/'commands.jsonl').open('a') as f:f.write(json.dumps(cmd)+'\n')
        subprocess.run(cmd,check=True)
    dirs=[a.out/'runs'/f'{k}-{scene}' for k in ('base','off')]
    rows=[json.loads((d/'row.json').read_text()) for d in dirs]
    files=[{f.name:hashlib.file_digest(f.open('rb'),'sha256').hexdigest() for f in (d/'output').glob('*.vtu')} for d in dirs]
    passed=all(r['exit_code']==0 and not r['timed_out'] for r in rows) and bool(files[0]) and files[0]==files[1]
    results.append(dict(scene=scene,passed=passed,file_hashes=files,exits=[r['exit_code'] for r in rows]))
    (a.out/'identity.json').write_text(json.dumps(results,indent=2)+'\n')
    print(scene,'PASS' if passed else 'FAIL',len(files[0]),'VTU files',flush=True)
raise SystemExit(0 if all(r['passed'] for r in results) else 1)
