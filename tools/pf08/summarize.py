"""Collect compact PF-08 measurements, preserving failures and partial runs."""
import argparse
import json
from pathlib import Path

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('root',type=Path)
a=p.parse_args();root=a.root
runs=json.loads((root/'probe-validated/probe-results.json').read_text())
solves=[s for r in runs for s in r.get('solves', []) if 'free_residual' in s]
summary=dict(provenance=json.loads((root/'probe-validated/provenance.json').read_text()),
    probe=dict(runs=len(runs),passed=sum(r['passed'] for r in runs),completed_solves=len(solves),
        maxima={k:max((s[k] for s in solves), default=None) for k in ('bc_error','free_residual','reaction_balance','displacement_error')},
        minimum_gap=min((s['gap_over_dhat'] for s in solves), default=None),
        max_manufactured_force_fd_error=max((s['manufactured_force_fd_error'] for r in runs for s in r.get('solves', [])), default=None),
        max_work_error=max((r['conservative_work'][-1]['error'] for r in runs if r.get('conservative_work')), default=None),
        max_friction_derivative_error=max((f['derivative_error'] for r in runs for f in r.get('friction', [])), default=None),
        max_friction_plateau_error=max((f['plateau_error'] for r in runs for f in r.get('friction', []) if f['plateau_error'] is not None), default=None),
        min_sliding_dissipation=min((f['dissipation'] for r in runs for f in r.get('friction', []) if abs(f['slip'])>.001), default=None),
        failures=[dict(length_scale=r['length_scale'],objective_scale=r['objective_scale'],steps=r['steps'],moving=r['moving'],coupled=r['coupled'],
            process_error=r.get('error'), exit_code=r['exit_code'], failed_solves=[s for s in r.get('solves', []) if not s['passed']]) for r in runs if not r['passed']]))
# Compare only completed endpoints, never a partial solution against t=1.
groups={}
for r in runs:
    if not r['passed']: continue
    groups.setdefault((r['coupled'],r['moving']),[]).append(r.get('solves', [])[-1]['normalized_energy'])
summary['probe']['normalized_endpoint_energy_spread']={str(k):max(v)-min(v) for k,v in groups.items()}
summary['scene_geometry']=json.loads((root/'scenes/scene-summary.json').read_text())
summary['fem']=json.loads((root/'scenes/fem-measurements.json').read_text())
if (root/'active-floor/probe-results.json').exists():
    summary['active_floor']=json.loads((root/'active-floor/probe-results.json').read_text())
(root/'measurements.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary['probe'],indent=2))
