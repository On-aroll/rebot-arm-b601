"""Check nominal view sampling against target-present/absent counterfactuals."""
import json
from pathlib import Path
import shutil

from risk_observation_dataset import views
from sim import ROOT, Simulation
from task_runner import digest, write


def main():
    root = (ROOT/"outputs/risk-observation-smoke-01").resolve()
    assert json.loads((root/"audit.json").read_text())["passed"]
    output = root/"sampler_independence.json"
    if output.exists():
        raise FileExistsError(output)
    config = json.loads((root/"config.json").read_text())
    seeds = [s for block in config["seed_blocks"].values() for s in block]
    sim = Simulation(ROOT/"assets/task_scene.xml")
    try:
        for seed in seeds:
            sim.reset(seed, absent=False)
            present = views(sim, seed)
            sim.reset(seed, absent=True)
            absent = views(sim, seed)
            assert present == absent and sim.data.time == 0.
    finally:
        sim.close()
    source = Path(__file__)
    shutil.copy2(source, root/"sampler_independence_source.py")
    write(output, {"passed": True, "seeds": seeds, "view_pairs_checked": len(seeds)*7,
        "source_sha256": digest(source), "sampler_source_sha256": digest(ROOT/"risk_observation_dataset.py"),
        "simulation_source_sha256": digest(ROOT/"sim.py"), "physics_steps": 0,
        "scope": "On all six smoke seeds, exact anchors/offsets/IK outcomes and joint initializations match with target present versus reset outside the scene. This checks object-placement independence of the initialized sampler, not physical view reachability or a learned control policy."})
    print(json.dumps({"passed": True, "view_pairs_checked": len(seeds)*7}))


if __name__ == "__main__":
    main()
