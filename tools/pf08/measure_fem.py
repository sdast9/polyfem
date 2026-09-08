"""Independent P1 tetrahedral Neo-Hookean measurements on PF-08 cube VTUs.

Reconstruct F, energy and elastic nodal forces from rest coordinates and saved
nodal displacements. Never interpret elastic residuals at contact nodes as the
full residual. Transient forces omit inertia and are labeled accordingly.
Requires NumPy; supports only the uncompressed inline binary fixture format.
"""
import argparse
import base64
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def arrays(path):
    root = ET.parse(path).getroot()
    if root.get('compressor') or root.get('header_type') != 'UInt64':
        raise ValueError(f'Unsupported VTU: {path}')
    result = {}
    for a in root.iter('DataArray'):
        if a.get('format') != 'binary':
            raise ValueError('Expected binary array')
        dtype = {'Float64': '<f8', 'Int64': '<i8', 'UInt8': 'u1'}[a.get('type')]
        payload = base64.b64decode(a.text.strip())
        if int.from_bytes(payload[:8], 'little') != len(payload)-8:
            raise ValueError('Invalid byte count')
        v = np.frombuffer(payload, dtype=dtype, offset=8)
        nc = int(a.get('NumberOfComponents', 1))
        result[a.get('Name', 'points')] = v.reshape(-1, nc) if nc > 1 else v
    return result


def measure(path, young, nu, time):
    data = arrays(path)
    points, u = data['points'], data['displacement']
    ends = data['offsets']; starts = np.r_[0, ends[:-1]]
    tets = np.array([data['connectivity'][a:b] for a,b,k in zip(starts,ends,data['types']) if k == 10])
    if tets.shape[1] != 4: raise ValueError('P1 tetrahedra required')
    # Output duplicates vertices by element. Merge by exact rest coordinates.
    # The fixtures use dyadic coordinates, so no proximity tolerance is needed.
    ids = np.unique(tets)
    unique, inverse = np.unique(points[ids], axis=0, return_inverse=True)
    mapping = np.full(len(points), -1, dtype=int); mapping[ids] = inverse
    forces = np.zeros_like(unique)
    mu, lam = young/(2*(1+nu)), young*nu/((1+nu)*(1-2*nu))
    energy, volume, dets = 0., 0., []
    max_F_error = 0.
    saved = np.stack([data[f'F_{i}'] for i in (1,2,3)], axis=1)
    for tet in tets:
        X = points[tet]; x = X+u[tet]
        D = (X[1:]-X[0]).T; inv = np.linalg.inv(D)
        vol = abs(np.linalg.det(D))/6
        F = (x[1:]-x[0]).T @ inv
        J = np.linalg.det(F)
        if not np.isfinite(J) or J <= 0: raise ValueError(f'Invalid tetrahedron in {path}')
        dets.append(J)
        max_F_error = max(max_F_error, float(np.max(np.abs(saved[tet]-F.T))))
        P = mu*(F-np.linalg.inv(F).T)+lam*np.log(J)*np.linalg.inv(F).T
        grads = np.vstack((-inv.sum(axis=0), inv))
        np.add.at(forces, mapping[tet], vol*(P @ grads.T).T)
        energy += vol*(mu/2*(np.sum(F*F)-3-2*np.log(J))+lam/2*np.log(J)**2)
        volume += vol
    if abs(volume-1.) > 1e-10 or max_F_error > 1e-10:
        raise ValueError(f'Fixture reconstruction mismatch: {path}')
    top = unique[:,2] == 1
    middle = (unique[:,2] > 0) & (unique[:,2] < 1)
    boundary_ids = ids[points[ids,2] == 1]
    bc_error = float(np.max(np.abs(u[boundary_ids]-np.array([0,0,-.25*time]))))
    return dict(step=int(path.stem.split('_')[1]),time=time,volume=volume,
                tet_count=len(tets),node_count=len(unique),min_det_F=min(dets),
                saved_F_max_difference=max_F_error,bc_error=bc_error,
                elastic_energy=energy,top_reaction=forces[top].sum(axis=0).tolist(),
                elastic_interior_force_norm=float(np.linalg.norm(forces[middle])),
                elastic_interior_force_over_E=float(np.linalg.norm(forces[middle])/young),
                elastic_net_force_norm=float(np.linalg.norm(forces.sum(axis=0))))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);a=p.parse_args()
    summaries=[]
    for record in json.loads((a.root/'scene-results.json').read_text()):
        run=a.root/record['scene']/record['mode']
        cfg=json.loads((run/'input/params.json').read_text()); mat=cfg['materials']
        frames=[]
        for path in sorted((run/'output').glob('step_*.vtu'),key=lambda p:int(p.stem.split('_')[1])):
            step=int(path.stem.split('_')[1])
            frames.append(measure(path,mat['E'],mat['nu'],step*record['dt']))
        work=0.
        for prev,cur in zip(frames,frames[1:]):
            work += .5*(prev['top_reaction'][2]+cur['top_reaction'][2])*(-.25)*(cur['time']-prev['time'])
        end=frames[-1]
        summary=dict(scene=record['scene'],exit_code=record['exit_code'],frames=frames,
            quasistatic=cfg['time']['quasistatic'],top_work_trapezoid=work,
            elastic_energy_change=end['elastic_energy']-frames[0]['elastic_energy'],
            work_minus_elastic_energy=work-end['elastic_energy']+frames[0]['elastic_energy'],
            work_balance_scope='Incomplete: excludes barrier energy/retuning work, friction dissipation and transient kinetic energy')
        summaries.append(summary)
    (a.root/'fem-measurements.json').write_text(json.dumps(summaries,indent=2)+'\n')
    for s in summaries:
        f=s['frames'][-1]
        print(s['scene'],f['step'],f['bc_error'],f['elastic_interior_force_over_E'],f['elastic_energy'],s['work_minus_elastic_energy'])

if __name__ == '__main__': main()
