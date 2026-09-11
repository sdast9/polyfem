#!/usr/bin/env python3
"""Summarize RB-14 FEM cases without changing calibration or solver outcomes."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'rb13'))
from reference_probe import shape

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--input',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
if a.output.exists():raise SystemExit('output exists; preserve prior evidence')
rows=[json.loads(f.read_text()) for f in sorted(a.input.glob('case-*/result.json'))]
assert len(rows)==18 and all(r['passed'] for r in rows)
def candidate(row,method):return next(c for c in row['candidates'] if c['method']==method)
train=[r for r in rows if r['config']['split']=='calibration']
ratios=[r['reference']['K']/candidate(r,'radius1')['K'] for r in train]
errors=[(r['reference']['p']-candidate(r,'radius1')['p'])/.1 for r in train]
envelope={'K_ratio':[min(ratios),max(ratios)],'p_error_over_dhat':[min(errors),max(errors)]}
stage1=json.loads(Path(__file__).with_name('results-20260910-stage1.json').read_text())['envelope']
def coverage(row,bounds):
 c=candidate(row,'radius1');ref=row['reference']
 kl,ku=[c['K']*v for v in bounds['K_ratio']]
 pl,pu=[c['p']+.1*v for v in bounds['p_error_over_dhat']]
 kc=kl*(1-1e-10)<=ref['K']<=ku*(1+1e-10);pc=pl-1e-11<=ref['p']<=pu+1e-11
 lower=ku*max(.045-pl,0)/shape(.045,.1)[1]
 upper=kl*max(.055-pu,0)/shape(.055,.1)[1]
 status='inactive/mixed' if pu>=.055 else ('empty' if lower>upper else 'active')
 return dict(K_covered=kc,p_covered=pc,joint_covered=kc and pc,K_interval=[kl,ku],p_interval=[pl,pu],k_interval=[lower,upper],status=status)
compact=[]
for row in rows:
 result={k:v for k,v in row.items() if k not in ['snapshot','candidates']}
 result['coverage_stage2']=coverage(row,envelope);result['coverage_stage1']=coverage(row,stage1)
 cs=[]
 for c in row['candidates']:
  cc={k:v for k,v in c.items() if k!='nonlinear'}
  nl={k:v for k,v in c['nonlinear'].items() if k not in ['full_noncontact_gradient','contact_gradient','displacement']}
  if 'gap' in nl:cc['nonlinear_target_error_over_dhat']=(nl['gap']-.05)/.1
  cc['nonlinear']=nl
  if c['method']!='current_trim1':
   ref=row['reference'];exact=ref['K']*max(.05-ref['p'],0)
   cc['demand_error_over_K_dhat']=(c['K']*max(.05-c['p'],0)-exact)/(ref['K']*.1)
  cs.append(cc)
 result['candidates']=cs
 if 'stale_reuse' in result:result['stale_reuse']={k:v for k,v in result['stale_reuse'].items() if k not in ['full_noncontact_gradient','contact_gradient','displacement']}
 compact.append(result)
summary={}
for split in ['calibration','holdout']:
 rs=[r for r in compact if r['config']['split']==split]
 summary[split]={'fixtures':len(rs),'coverage':{source:{key:sum(r[source][key] for r in rs) for key in ['K_covered','p_covered','joint_covered']} for source in ['coverage_stage2','coverage_stage1']},
 'intervals':dict(Counter(r['coverage_stage2']['status'] for r in rs)),
 'outcomes':dict(Counter(c['nonlinear']['status'] for r in rs for c in r['candidates'])),
 'methods':{method:{'max_abs_K_relative_error':max(abs(candidate(r,method)['K_relative_error']) for r in rs),
 'max_abs_p_error_over_dhat':max(abs(candidate(r,method)['p_error_over_dhat']) for r in rs),
 'max_abs_nonlinear_target_error_over_dhat':max((abs(candidate(r,method)['nonlinear_target_error_over_dhat']) for r in rs if 'nonlinear_target_error_over_dhat' in candidate(r,method)),default=None)} for method in ['full','radius0','radius1','radius2','diagonal','direction']}}
result={'scope':'RB-14 stage 2, P1 Neo-Hookean selected-contact FEM fixtures',
 'total_checks':sum(r['checks'] for r in rows),'fixture_count':len(rows),'stage2_empirical_envelope':envelope,
 'stage1_empirical_envelope':stage1,'summary':summary,'cases':compact}
a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
print(json.dumps({k:result[k] for k in ['total_checks','fixture_count','stage2_empirical_envelope','summary']},indent=2))
