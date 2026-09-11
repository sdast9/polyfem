#!/usr/bin/env python3
"""RB-14 stage 3 summary: preserve failures and separate prior challenges/new holdouts."""
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
if a.output.exists():raise SystemExit('output exists')
rows=[json.loads(f.read_text()) for f in sorted(a.input.glob('case-*/result.json'))]
assert len(rows)==21 and all(r['passed'] for r in rows)
def cand(row,name):return next(c for c in row['candidates'] if c['method']==name)
train=[r for r in rows if r['config']['split']=='calibration']
envelopes={}
for method in ['physical05','physical05_shared']:
 ratios=[r['reference']['K']/cand(r,method)['K'] for r in train]
 errors=[(r['reference']['p']-cand(r,method)['p'])/.1 for r in train]
 envelopes[method]={'K_ratio':[min(ratios),max(ratios)],'p_error_over_dhat':[min(errors),max(errors)]}
def coverage(row,method):
 c=cand(row,method);b=envelopes[method];ref=row['reference']
 kl,ku=[c['K']*v for v in b['K_ratio']];pl,pu=[c['p']+.1*v for v in b['p_error_over_dhat']]
 kc=kl*(1-1e-10)<=ref['K']<=ku*(1+1e-10);pc=pl-1e-11<=ref['p']<=pu+1e-11
 low=ku*max(.045-pl,0)/shape(.045,.1)[1];high=kl*max(.055-pu,0)/shape(.055,.1)[1]
 status='inactive/mixed' if pu>=.055 else ('empty' if low>high else 'active')
 return dict(K_covered=kc,p_covered=pc,joint_covered=kc and pc,k_interval=[low,high],status=status)
compact=[]
for r in rows:
 out={k:v for k,v in r.items() if k not in ['snapshot','candidates','stale_reuse']}
 out['coverage']={m:coverage(r,m) for m in envelopes}
 out['candidates']=[]
 for c in r['candidates']:
  cc={k:v for k,v in c.items() if k!='nonlinear'}
  cc['nonlinear']={k:v for k,v in c['nonlinear'].items() if k not in ['full_noncontact_gradient','contact_gradient','displacement']}
  if 'gap' in cc['nonlinear']:cc['nonlinear_target_error_over_dhat']=(cc['nonlinear']['gap']-.05)/.1
  out['candidates'].append(cc)
 if 'stale_reuse' in r:out['stale_reuse']={k:v for k,v in r['stale_reuse'].items() if k not in ['full_noncontact_gradient','contact_gradient','displacement']}
 compact.append(out)
summary={}
for split in ['calibration','prior_challenge','holdout']:
 rs=[r for r in compact if r['config']['split']==split]
 summary[split]={'fixtures':len(rs),'outcomes':dict(Counter(c['nonlinear']['status'] for r in rs for c in r['candidates'])),
 'coverage':{m:{key:sum(r['coverage'][m][key] for r in rs) for key in ['K_covered','p_covered','joint_covered']} for m in envelopes},
 'intervals':{m:dict(Counter(r['coverage'][m]['status'] for r in rs)) for m in envelopes},
 'methods':{m:{'max_abs_K_relative_error':max(abs(cand(r,m)['K_relative_error']) for r in rs),
 'max_abs_p_error_over_dhat':max(abs(cand(r,m)['p_error_over_dhat']) for r in rs),
 'max_abs_nonlinear_target_error_over_dhat':max((abs(cand(r,m)['nonlinear_target_error_over_dhat']) for r in rs if 'nonlinear_target_error_over_dhat' in cand(r,m) and cand(r,m)['nonlinear']['status']=='converged'),default=None)} for m in ['full','radius1','physical025','physical05','physical1','physical05_shared']}}
# Unchanged stage-2 controls are compared by value, not by timing or assertion count.
prior=json.loads(Path(__file__).with_name('results-20260910-stage2.json').read_text())['cases']
control_checks=0
for old in prior:
 new=next(r for r in compact if r['config']['name']==old['config']['name'])
 for m in ['full','radius1','current_trim1']:
  before,after=cand(old,m),cand(new,m)
  for key in ['K','p','k']:
   assert abs(before[key]-after[key])/(1+abs(before[key]))<=1e-8;control_checks+=1
  assert before['nonlinear']['status']==after['nonlinear']['status'];control_checks+=1
  if 'gap' in before['nonlinear']:
   assert abs(before['nonlinear']['gap']-after['nonlinear']['gap'])/.1<=1e-8;control_checks+=1
base=next(r for r in compact if r['config']['name']=='baseline')
boundary={'derivation':'At fixed H, k_target tends to zero as positive demand tends to zero; both proposed zero-demand branches instead return a finite coefficient.',
 'baseline_K':base['reference']['K'],'compressed_side_k_limit':0.,'zero_demand_current_k':base['current_assignment']['coefficient_trim1'],
 'zero_demand_tangent_k':base['reference']['K']/base['unit_curvature_at_target'],'simulation':'not run: algebraic branch-limit calculation'}
incomplete=[{'case':r['config']['name'],'method':c['method'],'endpoint':c['nonlinear']} for r in compact for c in r['candidates'] if c['nonlinear']['status'] not in ['converged','zero demand: unprotected solve not run']]
result={'scope':'RB-14 stage 3 physical neighborhoods/shared prediction; no production policy',
 'total_checks':sum(r['checks'] for r in rows),'control_comparisons_passed':control_checks,
 'fixture_count':len(rows),'incomplete_solves':incomplete,'empirical_envelopes':envelopes,'summary':summary,'zero_demand_branch_limit':boundary,'cases':compact}
a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
print(json.dumps({k:result[k] for k in ['total_checks','control_comparisons_passed','empirical_envelopes','summary','zero_demand_branch_limit']},indent=2))
