"""RB-04 saved-fixture discrete budgets; no solver execution or inferred contact path work."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'pf08'))
from measure_fem import arrays
from work_reference import mass_dot


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def elastic_segment(data, u0, u1, young, nu):
    """Independent P1 Neo-Hookean stress contraction along a straight displacement path."""
    tets = np.array([data['connectivity'][a:b] for a,b,k in zip(
        np.r_[0,data['offsets'][:-1]],data['offsets'],data['types']) if k==10])
    X=data['points'][tets]
    D=np.transpose(X[:,1:]-X[:,:1],(0,2,1))
    inv=np.linalg.inv(D); vol=np.abs(np.linalg.det(D))/6
    def deformation(u):
        y=X+u[tets]
        return np.transpose(y[:,1:]-y[:,:1],(0,2,1))@inv
    F0,F1=deformation(u0),deformation(u1); dF=F1-F0
    mu=young/(2*(1+nu)); lam=young*nu/((1+nu)*(1-2*nu))
    def evaluate(t):
        F=F0+t*dF; J=np.linalg.det(F)
        if np.any(J<=0): raise ValueError('Nonpositive path det(F) at quadrature sample')
        invT=np.transpose(np.linalg.inv(F),(0,2,1)); logJ=np.log(J)
        P=mu*(F-invT)+lam*logJ[:,None,None]*invT
        energy=np.sum(vol*(mu/2*(np.sum(F*F,axis=(1,2))-3-2*logJ)+lam/2*logJ**2))
        work=np.sum(vol*np.sum(P*dF,axis=(1,2)))
        return float(energy),float(work),float(J.min())
    e0,_,_=evaluate(0); e1,right,_=evaluate(1)
    estimates={}; errors={}; mindet=1e100
    for order in (2,4,8):
        nodes,weights=np.polynomial.legendre.leggauss(order)
        samples=[evaluate((t+1)/2) for t in nodes]
        work=sum(w*v[1]/2 for w,v in zip(weights,samples))
        estimates[str(order)]=work;errors[str(order)]=work-(e1-e0)
        mindet=min(mindet,min(v[2] for v in samples))
    # Same predeclared 1e-9 relative+absolute energy screen as prior references.
    assert abs(errors['8']) < 1e-9*(1+abs(e1-e0)), errors
    return dict(energy_start=e0,energy_end=e1,right_work=right,
        path_work=estimates,path_error=errors,sampled_path_min_det_F=mindet)


def analyze(run, metadata):
    cfg=json.loads((run/'params.json').read_text()); mat=cfg['materials'];dt=cfg['time']['dt']
    assert mat['type']=='NeoHookean' and mat['rho']==1000
    assert cfg['time'].get('quasistatic',False)==metadata['quasistatic']
    path=run/'output/physical-diagnostics.jsonl'
    records=[json.loads(line) for line in path.read_text().splitlines()]
    events=run/'output/coefficient-events.jsonl'
    first=json.loads(events.read_text().splitlines()[0])
    assert not np.any(first['coordinates']) and first['after']['objective']==0
    data0=arrays(run/'output/step_0.vtu');prev_u=data0['displacement'];prev_v=np.zeros_like(prev_u)
    assert not np.any(prev_u)
    prev_x=np.array(first['coordinates']);prev_E=prev_B=prev_K=0.
    totals={k:0. for k in ('support_work','elastic_change','barrier_change','kinetic_change',
        'IE_dissipation','friction_work_solved_lag','friction_work_updated_lag',
        'elastic_right_remainder','contact_right_remainder','parameter_energy',
        'equilibrium_work_defect','closure_error')}
    result=dict(run=metadata['evidence_subpath'],dt=dt,quasistatic=metadata['quasistatic'],
        friction_coefficient=metadata['friction_coefficient'],frames=[],source_hashes={
            'params':digest(run/'params.json'),'diagnostics':digest(path),'coefficient_events':digest(events)})
    for r in records:
        if r['outcome']!='accepted':
            result.setdefault('failed_attempts',[]).append({k:r[k] for k in ('step','outcome','error')});continue
        step=r['step']; data=arrays(run/f'output/step_{step}.vtu')
        assert np.array_equal(data['points'],data0['points']) and np.array_equal(data['connectivity'],data0['connectivity'])
        x=np.array(r['endpoint']['value']);dx=x-prev_x
        gradients={f['name']:np.array(f['gradient_force_units']['value']) for f in r['forms']}
        for name,g in gradients.items():
            if name not in ('elastic','barrier-contact','friction','inertia'): assert np.linalg.norm(g)<1e-10,name
        reaction=np.array(r['reactions'][0]['full_dof_vector']['value'])
        W=float(reaction@dx)
        elastic=elastic_segment(data,prev_u,data['displacement'],mat['E'],mat['nu'])
        E=r['elastic_energy']['value'];B=r['barrier_energy']['value']
        assert abs(elastic['energy_end']-E)<1e-9*(1+abs(E))
        assert abs(elastic['energy_start']-prev_E)<1e-9*(1+abs(prev_E))
        Ce=float(gradients['elastic']@dx)
        assert abs(Ce-elastic['right_work'])<1e-9*(1+abs(Ce))
        Cb=float(gradients['barrier-contact']@dx)
        common=r['barrier_start_energy_with_endpoint_snapshot']['value']
        P=common-prev_B;motion=B-common
        Cf=post=0.
        if 'friction' in gradients:
            before=np.array(r['lagging']['friction_before_update']['gradient_force_units'])
            after=np.array(r['lagging']['friction_after_update']['gradient_force_units'])
            assert np.linalg.norm(after-gradients['friction'])<1e-9*(1+np.linalg.norm(after))
            Cf=float(before@dx);post=float(after@dx)
            # Recorded support forces remain suitable for this fixture only:
            # lag change has zero work on all DOFs carrying recorded reactions.
            delta_support=float(((before-after)*dx)[reaction!=0].sum())
            assert abs(delta_support)<1e-9,delta_support
            assert Cf>=-1e-9, Cf
        K=D=Ci=0.
        if 'inertia' in gradients:
            v=(data['displacement']-prev_u)/dt;dv=v-prev_v
            K=.5*mass_dot(data,v,v);D=.5*mass_dot(data,dv,dv)
            Ci=float(gradients['inertia']@dx)
            assert abs(K-r['kinetic_energy']['value'])<1e-9*(1+K)
            assert abs(Ci-(K-prev_K+D))<1e-9*(1+abs(Ci))
            prev_v=v
        defect=W-(Ce+Cb+Ci+Cf)
        terms=dict(support_work=W,elastic_change=E-prev_E,barrier_change=B-prev_B,
            kinetic_change=K-prev_K,IE_dissipation=D,friction_work_solved_lag=Cf,
            friction_work_updated_lag=post,elastic_right_remainder=Ce-(E-prev_E),
            contact_right_remainder=Cb-motion,parameter_energy=P,equilibrium_work_defect=defect)
        rhs=(terms['elastic_change']+terms['barrier_change']+terms['kinetic_change']+D+Cf
             +terms['elastic_right_remainder']+terms['contact_right_remainder']-P+defect)
        terms['closure_error']=W-rhs
        assert abs(W-rhs)<1e-9*(1+abs(W))
        for k,val in terms.items(): totals[k]+=val
        result['frames'].append(dict(step=step,terms=terms,elastic_reference=elastic))
        prev_x,prev_u,prev_E,prev_B,prev_K=x,data['displacement'],E,B,K
    result['totals']=totals if result['frames'] else None
    if not result['frames']: result['unavailable_reason']='No accepted endpoint; work totals not measured'
    result['complete']=len(result['frames'])==round(cfg['time']['tend']/dt)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--evidence',type=Path,required=True)
    p.add_argument('--paired-results',type=Path,default=Path(__file__).with_name('physical-state-pairs-results-20260909.json'))
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'analysis-at-start.py').write_bytes(Path(__file__).read_bytes())
    source=json.loads(a.paired_results.read_text())
    result=dict(analysis_sha256=digest(Path(__file__)),input_index_sha256=digest(a.paired_results),
        reference_hashes={name:digest(Path(__file__).with_name(name)) for name in ('work_reference.py',)},
        scope='Discrete right-endpoint budget, not a complete continuum energy balance. Contact remainder unresolved between quadrature and feature jumps. Friction uses preceding frozen lag.',runs=[])
    for row in source['runs']:
        run=a.evidence/row['evidence_subpath']
        try:r=analyze(run,row)
        except Exception as e:
            result['runs'].append(dict(run=row['evidence_subpath'],analysis_error=repr(e)))
            (a.output/'results.json').write_text(json.dumps(result,indent=2)+'\n');raise
        result['runs'].append(r)
        (a.output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
        print(r['run'],r['complete'],r['totals'],flush=True)

if __name__=='__main__':main()
