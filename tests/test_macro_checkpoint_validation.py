#!/usr/bin/env python3
"""Reject malformed macro-strain checkpoints without publishing continuation.

Run the [macro_checkpoint] unit case first, then invoke with:
  python test_macro_checkpoint_validation.py BINARY SOURCE_CHECKPOINT FRESH_OUTPUT
SOURCE_CHECKPOINT is its final periodic-contact/free-component full/step_1.h5.
Requires h5py and numpy. Retains all mutated inputs and process logs.
"""
from pathlib import Path
import sys, shutil, json, subprocess, os
import h5py
import numpy as np
binary=Path(sys.argv[1]).resolve(); source=Path(sys.argv[2]).resolve(); out=Path(sys.argv[3]).resolve();out.mkdir(exist_ok=False)
g='/checkpoint/state/fork/macro_strain'
cases=['missing_group','missing_gradient','wrong_version','nonfinite_gradient','wrong_shape','inconsistent_gradient','missing_multipliers','nonfinite_multipliers','owner_mismatch','reorder']
results=[]
for name in cases:
 p=out/(name+'.h5');shutil.copy2(source,p)
 with h5py.File(p,'r+') as f:
  if name=='missing_group':del f[g]
  elif name=='missing_gradient':del f[g+'/gradient']
  elif name=='wrong_version':f[g].attrs['layout_version']=99
  elif name=='nonfinite_gradient':f[g+'/gradient'][...]=np.nan
  elif name=='wrong_shape':del f[g+'/gradient'];f[g].create_dataset('gradient',data=np.zeros((1,1)))
  elif name=='inconsistent_gradient':f[g+'/gradient'][...]=np.ones(f[g+'/gradient'].shape)
  elif name=='missing_multipliers':del f[g+'/multipliers']
  elif name=='nonfinite_multipliers':f[g+'/multipliers'][...]=np.nan
  elif name=='owner_mismatch':del f[g+'/periodic_contact']
 cmd=[str(binary),'--checkpoint',str(p),'-o',str(out/name),'--max_threads','1','--log_level','error']
 if name=='reorder':cmd+=['--checkpoint-reorder']
 r=subprocess.run(cmd,capture_output=True,text=True,timeout=90,env=dict(os.environ,OMP_NUM_THREADS='1',MKL_CBWR='COMPATIBLE'))
 (out/(name+'.log')).write_text(r.stdout+r.stderr)
 assert r.returncode!=0,(name,'accepted malformed checkpoint')
 assert not list((out/name).glob('step_*.h5')),(name,'published invalid continuation')
 results.append({'name':name,'exit':r.returncode,'passed':True})
(out/'results.json').write_text(json.dumps(results,indent=2));print('10 malformed/reorder macro cases rejected without checkpoint publication')
