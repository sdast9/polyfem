#!/usr/bin/env python3
"""Use EF-01's driver with explicit workspace and all five public smoke scenes."""
import argparse, sys
from pathlib import Path
p=argparse.ArgumentParser(add_help=False)
p.add_argument('--workspace', type=Path, required=True)
a, rest=p.parse_known_args()
sys.path.insert(0,str(a.workspace/'polyfem/tools/ef01'))
import ef01_run
for key, name in [('smoke-adaptive','quasistatic-adaptive.json'),
                  ('smoke-alhess','quasistatic-semi-alhess.json'),
                  ('smoke-friction','quasistatic-semi-friction.json')]:
    ef01_run.qn_run.SCENES[key]=a.workspace/'polyfem/scenes/semi-implicit'
    ef01_run.qn_run.SCENE_FILE[key]=name
sys.argv=[sys.argv[0]]+rest
ef01_run.main()
