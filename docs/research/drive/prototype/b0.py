import time, sys
from run import *
M = [str(HERE / "models")]
w = flat_world("flat")
t = time.time()
out = run("dd_turn05", w, [[0, 0, 0], [1.0, 0, 0.5]], 3.0, models=M, quiet=False)
print("elapsed", time.time() - t)
