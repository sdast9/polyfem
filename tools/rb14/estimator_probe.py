#!/usr/bin/env python3
"""RB-14 stage 1: standalone quadratic estimator comparisons; no production policy."""
import argparse
import ctypes
import hashlib
import json
import math
from pathlib import Path
import platform
import statistics
import subprocess
import sys
import time
import tracemalloc

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'rb13'))
from reference_probe import solve_spd, dot

DEFAULT = dict(E=10., contrast=1., anisotropy=1., support=2., h=1., rho=1.,
               dt=.2, speed=.4, load=0., nodes=3)
METHODS = ('full', 'radius0', 'radius1', 'diagonal', 'direction', 'rayleigh_raw')


def matvec(a, x):
    return [dot(row, x) for row in a]


def norm(x):
    return max(map(abs, x), default=0.)


def fixture(overrides):
    p = DEFAULT | overrides
    n, h = p['nodes'], p['h']
    nd = 4*n
    a = [[0.]*nd for _ in range(nd)]
    def block(i, j, scale, angle):
        c, s = math.cos(angle), math.sin(angle)
        q = p['anisotropy']
        b = [[scale*(c*c+q*s*s), scale*(1-q)*c*s],
             [scale*(1-q)*c*s, scale*(s*s+q*c*c)]]
        for x in range(2):
            for y in range(2):
                a[2*i+x][2*i+y] += b[x][y]
                if j is not None:
                    a[2*j+x][2*j+y] += b[x][y]
                    a[2*i+x][2*j+y] -= b[x][y]
                    a[2*j+x][2*i+y] -= b[x][y]
    for side in range(2):
        scale = p['E']*h*(p['contrast'] if side else 1.)
        for i in range(n-1):
            block(side*n+i, side*n+i+1, scale, .2+.3*i+.4*side)
        block(0 if side == 0 else 2*n-1, None, p['support']*scale, .3)
    mass = p['rho']*h**3
    r, j = [0.]*nd, [0.]*nd
    normal = [.6, .8]
    for node in range(2*n):
        sign = 1 if node < n else -1
        for d in range(2):
            k = 2*node+d
            a[k][k] += mass/p['dt']**2
            r[k] = -mass/p['dt']*sign*p['speed']/2*normal[d]
    for d in range(2):
        j[2*(n-1)+d], j[2*n+d] = -normal[d], normal[d]
        r[d] -= p['load']*normal[d]
        r[2*(2*n-1)+d] += p['load']*normal[d]
    return dict(params=p, H=a, residual=r, J=j, gap=.08*h, dhat=.1*h, target=.05*h)


def estimate(f, method):
    a, r, j = f['H'], f['residual'], f['J']
    if r is None:
        raise ValueError('unavailable residual')
    if not all(math.isfinite(x) for row in a for x in row):
        raise ValueError('nonfinite H')
    n = len(j)
    indices = list(range(n))
    if method.startswith('radius'):
        radius = int(method[-1])
        m = f['params']['nodes']
        indices = [2*v+d for v in range(m-1-radius, m+radius+1) for d in range(2)]
    aj = [[a[i][k] for k in indices] for i in indices]
    jj, rr = [j[i] for i in indices], [r[i] for i in indices]
    if method == 'diagonal':
        if any(a[i][i] <= 0 for i in range(n)):
            raise ValueError('nonpositive diagonal')
        z, y = [j[i]/a[i][i] for i in range(n)], [r[i]/a[i][i] for i in range(n)]
    elif method in ('direction', 'rayleigh_raw'):
        jj2, curvature = dot(j, j), dot(j, matvec(a, j))
        if jj2 <= 0 or curvature <= 0:
            raise ValueError('invalid direction')
        k = curvature/jj2**2 if method == 'direction' else curvature/jj2
        return dict(K=k, p=f['gap']-jj2*dot(j, r)/curvature, dofs=n,
                    raw_rayleigh=curvature/jj2)
    else:
        z, y = solve_spd(aj, jj), solve_spd(aj, rr)
    compliance = dot(jj, z)
    if compliance <= 0:
        raise ValueError('zero gap derivative')
    return dict(K=1/compliance, p=f['gap']-dot(jj, y), dofs=len(indices))


def checked_fallback(f, cached_token, current_token, supported_map=True):
    """Measured proposal: never use the cached numeric estimate."""
    reason = 'stale' if cached_token != current_token else 'fresh'
    if not supported_map:
        return dict(status='unavailable', reason='unsupported map', estimate=None)
    try:
        return dict(status='recomputed', reason=reason, estimate=estimate(f, 'full'))
    except ValueError as error:
        return dict(status='unavailable', reason=str(error), estimate=None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ipc-source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    ipc = args.ipc_source.resolve()
    bridge = HERE.parent/'rb13/scope_bridge.cpp'
    libpath = out/'barrier.dylib'
    command = ['/usr/bin/c++', '-std=c++17', '-O2', '-shared', '-fPIC',
               '-I'+str(ipc/'src'), str(bridge), str(ipc/'src/ipc/barrier/barrier.cpp'),
               '-o', str(libpath)]
    sources = [Path(__file__), HERE/'stage1-protocol.md', bridge,
               HERE.parent/'rb13/reference_probe.py', ipc/'src/ipc/barrier/barrier.cpp',
               ipc/'src/ipc/barrier/barrier.hpp']
    provenance = dict(command=command, platform=platform.platform(), python=sys.version,
                      hashes={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    (out/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    run = subprocess.run(command, capture_output=True, text=True)
    (out/'compile.log').write_text(run.stdout+run.stderr)
    run.check_returncode()
    lib = ctypes.CDLL(str(libpath))
    force = lib.rb13_force
    force.argtypes, force.restype = [ctypes.c_double, ctypes.c_double], ctypes.c_double
    checks, roots = [], []
    def check(name, condition, value=None):
        checks.append(dict(name=name, passed=bool(condition), measured=value))
    def close(name, a, b):
        err = abs(a-b)/(1+abs(b))
        check(name, math.isfinite(err) and err <= 1e-8, err)
    def root(k, ref, h, label):
        if k == 0:
            if ref['p'] <= 0:
                return dict(status='unprotected invalid free gap', gap=None)
            return dict(status='zero-demand reference only', gap=ref['p'])
        lo, hi = 0., max(h, ref['p'])
        for _ in range(120):
            d = (lo+hi)/2
            residual = ref['K']*(d-ref['p'])-k*force(d, h)
            if residual > 0:
                hi = d
            else:
                lo = d
        d = (lo+hi)/2
        error = abs(ref['K']*(d-ref['p'])-k*force(d,h))/(ref['K']*(h+abs(ref['p']))+abs(k*force(d,h)))
        check(label+'/root residual', error <= 1e-8, error)
        row = dict(status='solved', gap=d, normalized_residual=error, bracket=[lo,hi])
        roots.append(dict(label=label, k=k, K=ref['K'], p=ref['p'], dhat=h, **row))
        return row
    sweeps = dict(contrast=[.1,1,10,100], anisotropy=[.1,1,10], support=[.1,2,100],
                  h=[.25,1,4], rho=[.01,1,100], dt=[.02,.2,2], speed=[0,.4,2], load=[0,1,10])
    calibration = [('baseline', {})]
    calibration += [(f'{key}={v}', {key:v}) for key, values in sweeps.items()
                    for v in values if v != DEFAULT[key]]
    holdout = [(f'heldout/{key}', {key:value}) for key,value in
               dict(contrast=1000., anisotropy=100., support=.01, h=.1, rho=.001,
                    dt=4., speed=4., load=100.).items()]
    holdout += [('heldout/combined',dict(contrast=1000.,anisotropy=100.,support=.01,rho=.001,load=10.)),
                ('heldout/topology',dict(nodes=4))]
    records=[]
    for split, configs in [('calibration', calibration), ('holdout', holdout)]:
        for name, config in configs:
            f=fixture(config); ref=estimate(f,'full'); h=f['dhat']; target=f['target']
            row=dict(name=name,split=split,fixture=f,reference=ref,estimates={})
            z,y=solve_spd(f['H'],f['J']),solve_spd(f['H'],f['residual'])
            for rhs,sol,tag in [(f['J'],z,'compliance'),(f['residual'],y,'predictor')]:
                err=norm([x-v for x,v in zip(matvec(f['H'],sol),rhs)])/(1+norm(rhs))
                check(name+'/'+tag+' solve',err<=1e-8,err)
            exact_demand=ref['K']*max(target-ref['p'],0.)
            for method in METHODS:
                est=estimate(f,method)
                demand=est['K']*max(target-est['p'],0.)
                k=demand/force(target,h)
                predicted=root(k,est,h,name+'/'+method+'/predicted')
                realized=root(k,ref,h,name+'/'+method+'/realized')
                est.update(K_relative_error=est['K']/ref['K']-1,
                           p_error_over_dhat=(est['p']-ref['p'])/h,
                           demand=demand,demand_error_scaled=(demand-exact_demand)/(ref['K']*h),
                           k=k,predicted=predicted,realized=realized)
                if demand>0:
                    check(name+'/'+method+'/predicted target',abs(predicted['gap']-target)/h<=1e-8)
                if realized['gap'] is not None:
                    d=realized['gap']; F=k*force(d,h) if k else 0.
                    u=[-yy+zz*F for yy,zz in zip(y,z)]
                    close(name+'/'+method+'/full gap',f['gap']+dot(f['J'],u),d)
                    balance=[a+b-c*F for a,b,c in zip(matvec(f['H'],u),f['residual'],f['J'])]
                    check(name+'/'+method+'/full balance',norm(balance)/(1+norm(f['residual'])+abs(F))<=1e-8)
                row['estimates'][method]=est
            ks=[row['estimates'][m]['K'] for m in ['full','radius1','radius0']]
            check(name+'/nested upper stiffness', ks[0]<=ks[1]*(1+1e-8) and ks[1]<=ks[2]*(1+1e-8),ks)
            close(name+'/action reaction x',sum(f['J'][::2]),0)
            close(name+'/action reaction y',sum(f['J'][1::2]),0)
            records.append(row)
    # Fit only from calibration records; holdout results cannot alter the ranges.
    train=[r for r in records if r['split']=='calibration']
    ratios=[r['reference']['K']/r['estimates']['radius1']['K'] for r in train]
    errors=[(r['reference']['p']-r['estimates']['radius1']['p'])/r['fixture']['dhat'] for r in train]
    envelope=dict(K_ratio=[min(ratios),max(ratios)],p_error_over_dhat=[min(errors),max(errors)])
    coverage=[]
    for row in records:
        est=row['estimates']['radius1']; ref=row['reference']; h=row['fixture']['dhat']
        kl,ku=[v*est['K'] for v in envelope['K_ratio']]
        pl,pu=[est['p']+h*v for v in envelope['p_error_over_dhat']]
        kc=kl*(1-1e-10)<=ref['K']<=ku*(1+1e-10)
        pc=pl-1e-10*h<=ref['p']<=pu+1e-10*h
        low=ku*max(.45*h-pl,0)/force(.45*h,h)
        high=kl*max(.55*h-pu,0)/force(.55*h,h)
        status='inactive or mixed' if pu>=.55*h else ('empty' if low>high else 'active')
        result=dict(name=row['name'],split=row['split'],K_covered=kc,p_covered=pc,
                    joint_covered=kc and pc,K_range=[kl,ku],p_range=[pl,pu],
                    coefficient_interval=[low,high],status=status)
        if status=='active':
            endpoints=[root(k,ref,h,row['name']+'/empirical band') for k in [low,high]]
            result['realized_endpoints']=endpoints
            result['realized_band_covered']=all(e['gap'] is not None and .45-1e-8<=e['gap']/h<=.55+1e-8 for e in endpoints)
        coverage.append(result)
    controls={}
    # Independent tiny matrices demonstrate that diagonal estimates have no fixed error sign.
    diagonal=[]
    for off in [-1.,1.]:
        f=dict(H=[[2.,off],[off,2.]],J=[1.,1.],residual=[0.,0.],gap=.08)
        exact,approx=estimate(f,'full'),estimate(f,'diagonal')
        diagonal.append(dict(off=off,exact=exact,diagonal=approx))
    check('diagonal both error signs', diagonal[0]['diagonal']['K']>diagonal[0]['exact']['K'] and diagonal[1]['diagonal']['K']<diagonal[1]['exact']['K'])
    controls['diagonal']=diagonal
    predictor_signs=[]
    for remote in [-1.,1.]:
        residual=[0.,remote]
        actual=.08-dot([1.,0.],solve_spd([[2.,-1.],[-1.,2.]],residual))
        local=.08-residual[0]/2
        predictor_signs.append(dict(remote_residual=remote,full_p=actual,local_p=local,error=local-actual))
    check('local predictor both error signs',predictor_signs[0]['error']<0<predictor_signs[1]['error'])
    controls['predictor_signs']=predictor_signs
    # Exact permutation: transform H,r,J together, not collision indices as FEM IDs.
    f=fixture({}); perm=list(reversed(range(len(f['J']))))
    moved=f|dict(H=[[f['H'][i][j] for j in perm] for i in perm],J=[f['J'][i] for i in perm],residual=[f['residual'][i] for i in perm])
    orig,permuted=estimate(f,'full'),estimate(moved,'full')
    close('permuted K',orig['K'],permuted['K']);close('permuted predictor',orig['p'],permuted['p'])
    controls['permutation']=dict(original=orig,permuted=permuted,permutation=perm)
    obstacles=[]
    # Hfull=[[6,2],[2,5]], residual=[-.3,0], prescribed obstacle c.
    for c in [0.,-.03]:
        reduced=dict(H=[[6.]],J=[-1.],residual=[2*c-.3],gap=.08+c)
        est=estimate(reduced,'full'); close('moving obstacle predictor',est['p'],.03+c+c/3)
        demand=est['K']*(.05-est['p'])
        k=demand/force(.05,.1)
        realized=root(k,est,.1,'obstacle/'+str(c))
        F=k*force(realized['gap'],.1)
        u=(.3-2*c-F)/6
        close('obstacle free balance',6*u+2*c-.3+F,0.)
        close('obstacle realized gap',.08+c-u,.05)
        close('obstacle contact reaction',-F+F,0.)
        obstacles.append(dict(prescribed=c,reference=est,demand=demand,k=k,
                              realized=realized,free_displacement=u,
                              contact_forces=[-F,F],required_obstacle_external_force=2*u+5*c-F))
    rigid=[]
    for b in [100.,10000.,1000000.]:
        finite=dict(H=[[6.,2.],[2.,b]],J=[-1.,1.],residual=[-.3,0.],gap=.08)
        est=estimate(finite,'full')
        close('finite rigid analytic K',est['K'],(6*b-4)/(b+10))
        rigid.append(dict(obstacle_support=b,reference=est,error_to_prescribed_K=est['K']/6-1))
    check('rigid limit improves',abs(rigid[-1]['error_to_prescribed_K'])<abs(rigid[0]['error_to_prescribed_K']))
    controls['finite_rigid_limit']=rigid
    controls['obstacles']=obstacles
    invalid=[]
    for label,override in [('singular',dict(H=[[0.]*12 for _ in range(12)])),
                           ('indefinite',dict(H=[[-1. if i==j else 0. for j in range(12)] for i in range(12)])),
                           ('nonfinite',dict(H=[[float('nan')]*12 for _ in range(12)])),
                           ('no residual',dict(residual=None)),('zero J',dict(J=[0.]*12))]:
        outcome=checked_fallback(f|override,('dt',.2),('dt',.2))
        check(label+' classified',outcome['status']=='unavailable' and outcome['estimate'] is None)
        invalid.append(dict(case=label,**outcome))
    for label,newfixture,newtoken,supported in [('stale dt',fixture(dict(dt=2.)),('dt',2.),True),
                                               ('stale map',moved,('map',2),True),
                                               ('unsupported map',f,('map',3),False)]:
        outcome=checked_fallback(newfixture,('old',0),newtoken,supported)
        check(label+' classified',outcome['status']==('recomputed' if supported else 'unavailable'))
        invalid.append(dict(case=label,**outcome))
    controls['fallback']=invalid
    changed=fixture(dict(dt=2.));fresh=estimate(changed,'full')
    old_k=orig['K']*(.05-orig['p'])/force(.05,.1)
    fresh_k=fresh['K']*(.05-fresh['p'])/force(.05,.1)
    controls['stale_dt_consequence']=dict(old_k=old_k,fresh_k=fresh_k,
        reused=root(old_k,fresh,.1,'stale dt/reused'),
        recomputed=root(fresh_k,fresh,.1,'stale dt/recomputed'))
    check('fresh fallback target',abs(controls['stale_dt_consequence']['recomputed']['gap']-.05)/.1<=1e-8)
    # Independent contacts: test a single raw/global k against exact per-fixture bands.
    active=[]
    for row in train:
        ref=row['reference'];h=row['fixture']['dhat']
        if ref['p']<.45*h and row['fixture']['params']['h']==1. and row['fixture']['params']['dt']==.2:
            active.append(dict(name=row['name'],K=ref['K'],p=ref['p'],h=h,
                               lower=ref['K']*(.45*h-ref['p'])/force(.45*h,h),
                               upper=ref['K']*(.55*h-ref['p'])/force(.55*h,h)))
    lower=max(active,key=lambda x:x['lower']); upper=min(active,key=lambda x:x['upper'])
    global_control=dict(lower_case=lower,upper_case=upper,interval=[lower['lower'],upper['upper']],
                        empty=lower['lower']>upper['upper'],endpoint_comparisons=[])
    for k in global_control['interval']:
        global_control['endpoint_comparisons'].append(dict(k=k,scenes=[dict(name=x['name'],result=root(k,x,x['h'],'global/'+x['name'])) for x in [lower,upper]]))
    check('global band incompatibility reproduced',global_control['empty'])
    controls['global']=global_control
    # Cost includes extraction and two independent factorizations; no reuse installed.
    costs={}
    for method in METHODS:
        times=[]
        for _ in range(15):
            start=time.perf_counter_ns(); estimate(f,method); times.append(time.perf_counter_ns()-start)
        tracemalloc.start();estimate(f,method);_,peak=tracemalloc.get_traced_memory();tracemalloc.stop()
        costs[method]=dict(median_ns=statistics.median(times),min_ns=min(times),peak_python_bytes=peak,
                           dofs=estimate(f,method)['dofs'])
    summary={}
    for split in ['calibration','holdout']:
        rows=[r for r in records if r['split']==split];cov=[r for r in coverage if r['split']==split]
        summary[split]=dict(fixtures=len(rows),coverage={key:sum(c[key] for c in cov) for key in ['K_covered','p_covered','joint_covered']},
                           methods={m:dict(max_abs_K_relative_error=max(abs(r['estimates'][m]['K_relative_error']) for r in rows),
                                           max_abs_p_error_over_dhat=max(abs(r['estimates'][m]['p_error_over_dhat']) for r in rows),
                                           max_abs_demand_error_scaled=max(abs(r['estimates'][m]['demand_error_scaled']) for r in rows)) for m in METHODS})
    result=dict(protocol='stage1-protocol.md',summary=summary,envelope=envelope,coverage=coverage,
                controls=controls,costs=costs,records=records,roots=roots,checks=checks,
                passed=sum(c['passed'] for c in checks),total=len(checks))
    (out/'results.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(f"{result['passed']}/{result['total']} checks passed; {len(records)} fixtures; {len(roots)} positive-k roots")
    if result['passed']!=result['total']:
        print(json.dumps([c for c in checks if not c['passed']],indent=2));raise SystemExit(1)


if __name__=='__main__':
    main()
