"""World layouts, each a module with build(): one per URC 2027 field mission
(MISSIONS, which the referee and the mission tests use) and test courses that
are not missions (COURSES)."""
from . import astrobiology, autonomy, delivery, equipment, proving_ground

MISSIONS = {m.KEY: m for m in (autonomy, equipment, delivery, astrobiology)}
COURSES = {m.KEY: m for m in (proving_ground,)}
