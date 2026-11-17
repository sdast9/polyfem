"""Independent CLI regression: MMG written coordinates == next loaded geometry.
Usage: python test_opt_remesh_restart.py BINARY FRESH_OUTPUT FIXTURE_DIRECTORY
Fixture directory contains run.json, state.json and triangle.obj.
Uses existing non-strict optimization convention; no production tolerance changes.
"""
from pathlib import Path
import json,subprocess,copy,time,sys
binary=Path(sys.argv[1]).resolve();out=Path(sys.argv[2]).resolve();out.mkdir(exist_ok=False)
fixture=Path(sys.argv[3]).resolve()
def msh_vertices(path):
 lines=path.read_text().splitlines();i=lines.index('$Nodes')+1;blocks,count,_,_=map(int,lines[i].split());i+=1;verts=[]
 for _ in range(blocks):
  dim,tag,param,n=map(int,lines[i].split());i+=1;assert param==0;i+=n
  verts += [[float(v) for v in lines[i+j].split()[:3]] for j in range(n)];i+=n
 assert len(verts)==count;return verts
def obj_vertices(path):return [[float(v) for v in l.split()[1:4]] for l in path.read_text().splitlines() if l.startswith('v ')]
variants={'plain':{},'scale':{'transformation':{'scale':[1.5,.8]}},'translated-rotated':{'transformation':{'translation':[2,3],'rotation':20}},'unit':{'unit':'cm'},'normalize':{'advanced':{'normalize_mesh':True},'transformation':{'scale':[2,3]}},'refine':{'n_refs':1},'dimensions':{'transformation':{'dimensions':[2,3]}}}
variants.update({'body17':{'volume_selection':17},'disabled-entry':{},'obstacle-entry':{}})
rows=[]
for name,patch in variants.items():
 p=out/name;p.mkdir();s=json.loads((fixture/'state.json').read_text());s['geometry'][0].update(patch);s['geometry'][0]['mesh']=str(fixture/'triangle.obj');
 if name=='disabled-entry':s['geometry'].append({'mesh':str(fixture/'triangle.obj'),'enabled':False,'transformation':{'scale':[9,9]}})
 if name=='obstacle-entry':s['geometry'].append({'mesh':str(fixture/'triangle.obj'),'is_obstacle':True,'transformation':{'translation':[10,10]},'surface_selection':2})
 s['geometry'][0]['surface_selection']=1;s['boundary_conditions']={'dirichlet_boundary':[{'id':'all','value':[0,0]}]};(p/'state.json').write_text(json.dumps(s));a=json.loads((fixture/'run.json').read_text());a['states'][0]['path']='state.json';a['parameters']='auto';a['variable_to_simulation'][0]['active_geometry_nodes']=[];a['functionals'].append({'type':'volume','state':0,'weight':.01});a['remeshing']['max_restarts']=2;a['output']['directory']=str(p/'output');(p/'run.json').write_text(json.dumps(a));t=time.monotonic()
 with (p/'run.log').open('w') as f:r=subprocess.run([str(binary),'-j',str(p/'run.json'),'--max_threads','1','--ns'],cwd=p,stdout=f,stderr=subprocess.STDOUT,timeout=90)
 checks=[]
 for k in (0,1):
  m=p/f'output/remesh_round_{k}/remeshed.msh';o=p/f'output/remesh_round_{k+1}/opt_state_0_iter_0.obj'
  if not m.exists() or not o.exists():checks.append({'round':k,'present':False});continue
  v,w=msh_vertices(m),obj_vertices(o);same=len(v)==len(w);error=max((abs(a-b) for x,y in zip(v,w) for a,b in zip(x,y)),default=0.) if same else None
  checks.append({'round':k,'present':True,'counts_equal':same,'max_coordinate_error':error,'pass':same and error<1e-12})
 rows.append({'name':name,'exit':r.returncode,'seconds':time.monotonic()-t,'checks':checks,'pass':r.returncode==0 and all(c.get('pass',False) for c in checks)})
 (out/'results.json').write_text(json.dumps(rows,indent=2));print(name,rows[-1]['pass'],flush=True)
raise SystemExit(0 if all(row['pass'] for row in rows) else 1)
