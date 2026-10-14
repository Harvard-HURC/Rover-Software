"""G2: the existing physics tests of the proving ground and the lander, run with DART's PGS solver
(the tests' world copies get <solver_type>pgs</solver_type>). Usage: python g2_tests.py <solver>"""
import contextlib
import functools
import json
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_lib as G  # noqa: E402

solver = sys.argv[1]
sys.path.insert(0, str(G.REPO / "sim" / "tests"))
import test_urc_sim  # noqa: E402
import test_urc_terrain  # noqa: E402
import worldfiles  # noqa: E402

if solver != "dantzig":
    test_urc_terrain.simulate = functools.partial(G.S.simulate, solver=solver)
    original = worldfiles.world_copy

    @contextlib.contextmanager
    def world_copy(*args, **kwargs):
        with original(*args, **kwargs) as path:
            text = open(path).read()
            with G.S.temp_sdf(G.S.variant_sdf(text, solver=solver)) as copy:
                yield copy
    test_urc_sim.world_copy = world_copy  # the lander test runs its copy in a TestFixture itself

suite = unittest.TestSuite()
loader = unittest.TestLoader()
suite.addTests(loader.loadTestsFromTestCase(test_urc_terrain.ProvingGroundPhysics))
suite.addTests(loader.loadTestsFromTestCase(test_urc_sim.Lander))
result = unittest.TextTestRunner(verbosity=2).run(suite)
out = dict(solver=solver, ran=result.testsRun, failures=[str(t) for t, _ in result.failures],
           errors=[str(t) for t, _ in result.errors],
           failure_text=[tb[-800:] for _, tb in result.failures + result.errors])
print("RESULT", json.dumps(out))
G.save(f"g2_tests_{solver}", out)
