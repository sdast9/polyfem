#!/usr/bin/env python3
"""Compile the isolated RB-14 FEM probe against an existing Makefiles build."""
import argparse, hashlib, json, pathlib, shlex, subprocess
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--build',type=pathlib.Path,required=True)
p.add_argument('--output',type=pathlib.Path,required=True)
p.add_argument('--config',type=pathlib.Path,required=True)
a=p.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
build=a.build.resolve()/'tests';source=pathlib.Path(__file__).with_name('fem_probe.cpp').resolve()
flags={}
for line in (build/'CMakeFiles/unit_tests.dir/flags.make').read_text().splitlines():
 if ' = ' in line:
  k,v=line.split(' = ',1);flags[k]=shlex.split(v)
link=shlex.split((build/'CMakeFiles/unit_tests.dir/link.txt').read_text());obj=out/'probe.o';binary=out/'probe'
compile=[link[0]]+flags['CXX_DEFINES']+flags['CXX_INCLUDES']+flags['CXX_FLAGS']+['-c',str(source),'-o',str(obj)]
link=[v for v in link if not v.endswith('.o')];link[link.index('-o')+1]=str(binary);link.insert(1,str(obj))
(out/'commands.json').write_text(json.dumps([compile,link],indent=2)+'\n')
(out/'fem_probe.cpp').write_bytes(source.read_bytes())
inputs=[source,build/'CMakeFiles/unit_tests.dir/flags.make',build/'CMakeFiles/unit_tests.dir/link.txt']
inputs += [(build/v).resolve() for v in link if v.endswith('.a')]
(out/'build-input-hashes.json').write_text(json.dumps({str(v):hashlib.sha256(v.read_bytes()).hexdigest() for v in inputs},indent=2)+'\n')

for name,cmd in [('compile',compile),('link',link)]:
 r=subprocess.run(cmd,cwd=build,text=True,capture_output=True);(out/(name+'.log')).write_text(r.stdout+r.stderr)
 (out/(name+'-exit.json')).write_text(json.dumps({'exit':r.returncode})+'\n')
 if r.returncode:raise SystemExit(f'{name} failed: {out/(name+".log")}')
config=a.config.resolve();(out/'config.json').write_bytes(config.read_bytes())
configs=json.loads(config.read_text());configs=configs if isinstance(configs,list) else [configs]
results=[];baseline_k=None
for index,cfg in enumerate(configs):
 case=out/f'case-{index:02d}';case.mkdir()
 if cfg.get('name')=='dt05' and baseline_k is not None:cfg=cfg|{'stale_k':baseline_k}
 configfile=case/'input.json';configfile.write_text(json.dumps(cfg,indent=2)+'\n')
 cmd=[str(binary),str(configfile)];r=subprocess.run(cmd,cwd=case,text=True,capture_output=True)
 (case/'result.json').write_text(r.stdout);(case/'stderr.log').write_text(r.stderr)
 (case/'run.json').write_text(json.dumps(dict(command=cmd,exit=r.returncode),indent=2)+'\n')
 try:result=json.loads(r.stdout)
 except json.JSONDecodeError:result={'passed':False,'error':'non-JSON output; inspect case log'}
 results.append(dict(name=cfg.get('name',str(index)),exit=r.returncode,passed=result.get('passed',False),checks=result.get('checks',0)))
 if cfg.get('name')=='baseline' and result.get('passed'):
  baseline_k=next(c['k'] for c in result['candidates'] if c['method']=='full')
 print(results[-1],flush=True)
(out/'provenance.json').write_text(json.dumps({'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),'config_sha256':hashlib.sha256(config.read_bytes()).hexdigest()},indent=2)+'\n')
(out/'run-summary.json').write_text(json.dumps(results,indent=2)+'\n')
raise SystemExit(0 if all(r['passed'] and r['exit']==0 for r in results) else 1)
