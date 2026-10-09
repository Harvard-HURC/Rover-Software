"""G7: the mission and terrain tests (no physics) on the scratch tree where Delivery and Astrobiology are
2049^2 (the other worlds generated unchanged beside them)."""
import json
import os
import subprocess
import sys


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_lib as G  # noqa: E402

SCRATCH = G.SCRATCH
tree = SCRATCH / "trees" / "g7_2049" / "sim"
gen = subprocess.run([sys.executable, str(tree / "gen_worlds.py"), "autonomy", "equipment_servicing", "proving_ground"],
                     capture_output=True, text=True)
print(gen.stdout[-600:], gen.stderr[-2000:])
code = """
import json, sys, unittest
sys.path.insert(0, '.')
import test_urc_missions, test_urc_terrain
loader = unittest.TestLoader()
suite = unittest.TestSuite()
suite.addTests(loader.loadTestsFromModule(test_urc_missions))
for name in dir(test_urc_terrain):
    case = getattr(test_urc_terrain, name)
    if isinstance(case, type) and issubclass(case, unittest.TestCase) and name != 'ProvingGroundPhysics':
        suite.addTests(loader.loadTestsFromTestCase(case))
r = unittest.TextTestRunner(verbosity=1).run(suite)
print('RESULT', json.dumps(dict(ran=r.testsRun, failures=[str(t) for t, _ in r.failures], errors=[str(t) for t, _ in r.errors],
                                 text=[tb[-1200:] for _, tb in r.failures + r.errors])))
"""
run = subprocess.run([sys.executable, "-c", code], cwd=tree / "tests", capture_output=True, text=True)
line = next((l for l in run.stdout.splitlines() if l.startswith("RESULT")), None)
print(run.stderr[-3000:])
out = json.loads(line[7:]) if line else dict(error=run.stdout[-2000:] + run.stderr[-2000:])
(SCRATCH / "results" / "g7_invariants.json").write_text(json.dumps(out, indent=1))
print(json.dumps(out, indent=1)[:3000])
