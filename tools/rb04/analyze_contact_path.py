"""Summarize frozen contact-path quadrature without treating detected events as exhaustive."""
import argparse
import hashlib
import json
from pathlib import Path


def analyze(run):
    rows=[json.loads(l) for l in (run/'output/physical-diagnostics.jsonl').read_text().splitlines()]
    result={'run':run.name,'steps':[]}
    for r in rows:
        if r['outcome']!='accepted':
            result.setdefault('failures',[]).append({'step':r['step'],'error':r['error']});continue
        p=r.get('contact_path',{})
        if 'samples' not in p:
            result['steps'].append({'step':r['step'],'unavailable':p});continue
        scale=r['acceleration_scaling']['value'];samples=p['samples']
        assert len(samples)==1025
        start=samples[0]['energy_objective']/scale;end=samples[-1]['energy_objective']/scale
        assert abs(start-r['barrier_start_energy_with_endpoint_snapshot']['value'])<1e-9*(1+abs(start))
        assert abs(end-r['barrier_energy']['value'])<1e-9*(1+abs(end))
        transitions=[]
        for t in p['transitions']:
            left,right=t['left'],t['right']; width=right['t']-left['t']
            assert 0<width<=2**-33
            smooth=.5*(left['directional_derivative_objective']+right['directional_derivative_objective'])*width/scale
            jump=t['energy_jump_estimate_objective']/scale
            transitions.append(dict(t_left=left['t'],t_right=right['t'],width=width,
                energy_jump=jump,smooth_bracket_work=smooth,jump_minus_smooth_bracket_work=jump-smooth,
                multiple_signatures=t['multiple_signatures_in_bracket'],
                left_signature=p['signatures'][left['signature']],right_signature=p['signatures'][right['signature']]))
        jump_sum=sum(t['jump_minus_smooth_bracket_work'] for t in transitions)
        q=[]
        for row in p['quadrature']:
            n=row['panels'];work=row['gradient_integral_objective']/scale
            # Independently reconstruct the nested trapezoids from raw samples.
            other=sum((.5 if i in (0,n) else 1)*samples[i*(1024//n)]['directional_derivative_objective']/scale/n for i in range(n+1))
            assert abs(other-work)<1e-9*(1+abs(work))
            simpson=sum((1 if i in (0,n) else 4 if i%2 else 2)*samples[i*(1024//n)]['directional_derivative_objective']/scale/(3*n) for i in range(n+1))
            q.append(dict(panels=n,gradient_work=work,simpson_gradient_work=simpson,
                energy_minus_simpson_work=end-start-simpson,energy_minus_work=end-start-work,
                energy_minus_work_minus_detected_jumps=end-start-work-jump_sum))
        result['steps'].append(dict(step=r['step'],start_energy=start,end_energy=end,energy_change=end-start,
            quadrature=q,detected_jump_sum=jump_sum,transitions=transitions,
            sampled_signature_count=len(p['signatures'])))
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('runs',type=Path,nargs='+');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result={'analysis_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'scope':'Frozen endpoint coefficient snapshot on actual straight displacement segments; detected jumps only. Multiple/undetected transitions may remain; no complete physical balance claim.',
        'runs':[analyze(run) for run in a.runs]}
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    for r in result['runs']:
        for s in r['steps']:
            print(r['run'],s['step'],s.get('quadrature',[]), 'detected jumps',s.get('detected_jump_sum'),flush=True)

if __name__=='__main__':main()
