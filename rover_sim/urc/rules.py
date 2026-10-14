"""Numbers from the URC 2027 rules ("University Rover Challenge 2027 -
Requirements and Guidelines") and the URC Q&A as of 2026-10-03.

Each constant names the rule it comes from. Values the rules leave open are
not here; the mission layouts choose them and say so.
"""

# --- General -----------------------------------------------------------------
ROVER_MAX_MASS = 50.0  # [kg] deployed, 2.a.iii
ROVER_BOX = 1.2  # [m] cube the stowed rover must fit in, 2.a.ii
ANTENNA_MAX_HEIGHT = 3.0  # [m] C2 base station antenna, 2.c.iv
ANTENNA_MAX_OFFSET = 5.0  # [m] antenna base from the C2 station, 2.c.iv

# --- Autonomy (1.e) ----------------------------------------------------------
LED_AUTONOMOUS = "red"  # 1.e.ii
LED_TELEOP = "blue"
LED_ARRIVED = "green"  # flashing
LED_STATES = ("off", LED_AUTONOMOUS, LED_TELEOP, LED_ARRIVED)

ASTRONAUT_TOLERANCE = 3.0  # [m] stop within, 1.e.v, 1.e.vi, 1.e.ix
STAY_DISTANCE = 20.0  # [m] astronaut walks more than this away, 1.e.vii
COME_MIN_DISTANCE = 20.0  # [m] astronaut at least this far for Fetch!/Come!, Q&A Autonomy 7
ASTRONAUT_COMMANDS = ("follow", "stay", "fetch", "come", "give")  # in order, Q&A Autonomy 13

ROUTE_TOLERANCE = 1.0  # [m] stop within, 1.e.xvii
ROUTE_POINTS_PER_TARGET = 25  # 1.e.xvii
# Utah state-owned square mile for route-finding targets, 1.e.xiii:
# ((south lat, west lon), (north lat, east lon)) in WGS84 degrees.
ROUTE_AREA = ((38.411, -110.786), (38.425, -110.768))

# AR posts, 1.e.xii: three 20 x 20 cm faces, 0.5-1.5 m off the ground, 4x4_50
# ArUco tag with a one-cell white border so cells are 2.5 cm.
AR_DICTIONARY = "DICT_4X4_50"
AR_FACE = 0.20  # [m]
AR_CELL = 0.025  # [m]
AR_FACES = 3
AR_MIN_HEIGHT = 0.5  # [m] marker above ground
AR_MAX_HEIGHT = 1.5
# Tag IDs decoded from the rules' figure (Start, Post 1, Post 2).
AR_START_ID = 0
AR_POST1_ID = 1
AR_POST2_ID = 2

# --- Equipment Servicing (1.d) -----------------------------------------------
LANDER_MAX_DISTANCE = 100.0  # [m] travel to the lander, 1.d.i
EQUIPMENT_MAX_HEIGHT = 1.5  # [m] everything to service is below this, 1.d.i
CACHE_HANDLE_MIN_LENGTH = 0.10  # [m] 1.d.ii
CACHE_HANDLE_MAX_DIAMETER = 0.05  # [m]
CACHE_MAX_MASS = 5.0  # [kg]
LAUNCH_KEY_LENGTH = (3, 6)  # letters, 1.d.ii autonomous typing
# Keyboard fiducials: 2 x 2 cm 4x4_50 tags at the keyboard corners; IDs decoded
# from the rules' figure (top-left, top-right, bottom-left, bottom-right).
KEYBOARD_TAG_SIZE = 0.02  # [m]
KEYBOARD_TAG_IDS = {"top_left": 1, "top_right": 4, "bottom_left": 2, "bottom_right": 3}
# Lock fiducials: 1 x 1 cm tags at the keyhole corners (same IDs) and on the key.
LOCK_TAG_SIZE = 0.01  # [m]
KEY_TAG_IDS = (5, 6)  # one per side of the key head
CAMLOCK_DIAMETER = 0.0381  # [m] 1.5 inch cam lock hose fitting
VALVE_TURN = 0.5 * 3.141592653589793  # [rad] quarter-turn lever valve
# Redragon K552 (linked in the rules): tenkeyless, 87 keys.
KEYBOARD_KEYS = 87
EPAPER_DIAGONAL = 4.3 * 0.0254  # [m] display linked in the rules, Q&A Equipment 3

# --- Delivery (1.c) ----------------------------------------------------------
DELIVERY_MAX_RANGE = 1000.0  # [m] from the start gate, 1.c.ii
OBJECT_MAX_MASS = 5.0  # [kg] 1.c.iii
OBJECT_MAX_SIZE = 0.40  # [m] each dimension
GRASP_MAX_DIAMETER = 0.075  # [m] graspable feature

# --- Astrobiology (1.b) ------------------------------------------------------
SITE_RADIUS = 500.0  # [m] sites within this of the C2 station, 1.b.ii
MIN_SITES = 2  # 1.b.vi
SAMPLE_DEPTH = 0.10  # [m] 1.b.vi
SAMPLE_MIN_MASS = 0.005  # [kg]
ROVING_TIME = (20 * 60, 30 * 60)  # [s] 1.b.ii

# --- Mission times -----------------------------------------------------------
AUTONOMY_TIME = 40 * 60  # [s] 1.e.i
EQUIPMENT_TIME = 30 * 60  # [s] 1.d.i
DELIVERY_TIME = (30 * 60, 60 * 60)  # [s] 1.c.i
