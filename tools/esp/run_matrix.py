#!/usr/bin/env python3
"""Run the ESP / IPC / semi-implicit comparison of docs/esp-assessment-20261025.md.

Standard library only. --upstream is an upstream PolyFEM_bin with ESP (polyfem
`cc1eac01e1`), --esp-data its esp_data/ directory, --fork a fork PolyFEM_bin and
--fork-scenes the fork's scenes/semi-implicit. --only filters labels by substring
(esp-suite/, fork-suite/, followup/, ldlt/).

usage: run_matrix.py --upstream BIN --fork BIN --esp-data DIR --fork-scenes DIR
                     --output DIR [--jobs N] [--only SUBSTR]
"""
import argparse, copy, json, os, shutil, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path


def merge(base, over):
    out = copy.deepcopy(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def drop(d, *path):
    for p in path[:-1]:
        d = d.get(p, {})
    d.pop(path[-1], None)


def esp_as_is(s):
    return s


def esp_kappa(k):
    def f(s):
        s = copy.deepcopy(s)
        s.setdefault("solver", {}).setdefault("contact", {})["barrier_stiffness"] = k
        return s
    return f


def to_upstream_ipc(s):
    s = copy.deepcopy(s)
    c = s.setdefault("contact", {})
    c["use_esp_formulation"] = False
    c.pop("barrier", None)
    c.pop("esp_params", None)
    drop(s, "solver", "contact", "barrier_stiffness")  # upstream default: adaptive
    return s


def to_fork_semi(s):
    s = copy.deepcopy(s)
    c = s.setdefault("contact", {})
    for k in ("use_esp_formulation", "barrier", "esp_params"):
        c.pop(k, None)
    drop(s, "solver", "nonlinear", "line_search", "accept_energy_decrease_in_grad_mode")
    s.setdefault("solver", {}).setdefault("contact", {})["barrier_stiffness"] = "semi_implicit"
    return s


def to_fork_semi_defaults(s):
    s = to_fork_semi(s)
    nl = s.get("solver", {}).pop("nonlinear", None)
    return s


def ldlt(f):
    def g(s):
        s = f(s)
        s.setdefault("solver", {}).setdefault("linear", {})["solver"] = "Eigen::SimplicialLDLT"
        return s
    return g


def fork_as_is(s):
    return s


def fork_to_upstream_esp(k):
    def f(s):
        s = copy.deepcopy(s)
        c = s.setdefault("contact", {})
        c["use_esp_formulation"] = True
        c["barrier"] = "normalized_log"
        c["esp_params"] = {"normalize_weights": False, "quadrature_order": 0}
        s.setdefault("solver", {}).setdefault("contact", {})["barrier_stiffness"] = k
        return s
    return f


ESP_SCENES = ["spikes", "sliding-wedge", "spike-in-crack", "boxes"]
FORK_SCENES = ["quasistatic-semi", "transient-semi", "quasistatic-semi-friction"]


def matrix(a):
    runs = []
    for sc in ESP_SCENES:
        runs.append((f"esp-suite/{sc}/U-ESP", "upstream", "esp", sc, esp_as_is))
        runs.append((f"esp-suite/{sc}/U-IPC", "upstream", "esp", sc, to_upstream_ipc))
        runs.append((f"esp-suite/{sc}/F-SEMI", "fork", "esp", sc, to_fork_semi))
    for k in (1e3, 1e5):
        runs.append((f"esp-suite/spike-in-crack/U-ESP-k{k:g}", "upstream", "esp", "spike-in-crack", esp_kappa(k)))
    for sc in ("spikes", "sliding-wedge", "spike-in-crack"):
        runs.append((f"followup/{sc}/F-SEMI-forkdefaults", "fork", "esp", sc, to_fork_semi_defaults))
    for i in (1, 2):
        runs.append((f"followup/spike-in-crack/U-ESP-repeat{i}", "upstream", "esp", "spike-in-crack", esp_as_is))
    for sc in ("spike-in-crack", "spike-in-crack-rigid"):
        runs.append((f"ldlt/{sc}/U-ESP", "upstream", "esp", sc, ldlt(esp_as_is)))
        runs.append((f"ldlt/{sc}/U-IPC", "upstream", "esp", sc, ldlt(to_upstream_ipc)))
        runs.append((f"ldlt/{sc}/F-SEMI-forkdefaults", "fork", "esp", sc, ldlt(to_fork_semi_defaults)))
    for sc in FORK_SCENES:
        runs.append((f"fork-suite/{sc}/F-SEMI", "fork", "fork", sc, fork_as_is))
        runs.append((f"fork-suite/{sc}/U-IPC", "upstream", "fork", sc, to_upstream_ipc))
        ks = (1e3, 1e4, 1e5) if sc == "quasistatic-semi" else (1e4,)
        for k in ks:
            runs.append((f"fork-suite/{sc}/U-ESP-k{k:g}", "upstream", "fork", sc, fork_to_upstream_esp(k)))
    return runs


def prepare(a, label, suite, scene, transform, root):
    rd = root / label
    (rd / "jsons").mkdir(parents=True)
    if suite == "esp":
        src = json.loads((a.esp_data / "jsons" / f"{scene}.json").read_text())
        common = json.loads((a.esp_data / "jsons" / "common.json").read_text())
        src.pop("common", None)
        scene_json = merge(common, src)
        (rd / "meshes").symlink_to(a.esp_data / "meshes")
    else:
        scene_json = json.loads((a.fork_scenes / f"{scene}.json").read_text())
        for asset in a.fork_scenes.iterdir():
            if asset.suffix in (".mesh", ".obj", ".msh"):
                shutil.copy(asset, rd / "jsons" / asset.name)
    scene_json = transform(scene_json)
    scene_json.setdefault("output", {})["json"] = "sim.json"
    (rd / "jsons" / "input.json").write_text(json.dumps(scene_json, indent=2))
    return rd


def run(a, item, root):
    label, which, suite, scene, transform = item
    rd = prepare(a, label, suite, scene, transform, root)
    binary = a.upstream if which == "upstream" else a.fork
    out = rd / "out"
    cmd = [str(binary), "-j", str(rd / "jsons" / "input.json"), "-o", str(out),
           "--max_threads", "1", "--log_level", "debug"]
    env = dict(os.environ, OMP_NUM_THREADS="1", MKL_CBWR="COMPATIBLE")
    t0 = time.monotonic()
    with open(rd / "log.txt", "wb") as log:
        try:
            p = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, env=env,
                               cwd=rd / "jsons", timeout=a.timeout)
            code = p.returncode
        except subprocess.TimeoutExpired:
            code = "timeout"
    wall = time.monotonic() - t0
    res = {"label": label, "binary": which, "scene": scene, "exit": code, "wall_s": round(wall, 2), "cmd": cmd}
    (rd / "run.json").write_text(json.dumps(res, indent=2))
    print(f"{label}: exit {code}, {wall:.1f}s", flush=True)
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--upstream", type=Path, required=True)
    ap.add_argument("--fork", type=Path, required=True)
    ap.add_argument("--esp-data", type=Path, required=True)
    ap.add_argument("--fork-scenes", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--timeout", type=float, default=3600)
    ap.add_argument("--only", default="")
    a = ap.parse_args()
    a.esp_data = a.esp_data.resolve(); a.fork_scenes = a.fork_scenes.resolve()
    a.output.mkdir(parents=True, exist_ok=False)
    items = [m for m in matrix(a) if a.only in m[0]]
    with ThreadPoolExecutor(a.jobs) as ex:
        results = list(ex.map(lambda it: run(a, it, a.output), items))
    (a.output / "runs.json").write_text(json.dumps(results, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
