"""Central configuration: every tunable number lives here."""
import math

# ---- Window and timing ----
WIDTH, HEIGHT = 1000, 700
FPS = 30
DT = 1.0 / FPS          # fixed time step (seconds) so physics is frame-rate independent

# ---- Colors (R, G, B) ----
BG = (30, 33, 40)
WALL = (90, 95, 110)
STATIC_OBS = (200, 80, 80)
MOVING_OBS = (240, 170, 50)
CAR = (60, 160, 255)
GOAL = (60, 220, 120)
START = (180, 180, 180)
TEXT = (235, 235, 235)

# ---- Vehicle (bicycle model, units: pixels and seconds) ----
WHEELBASE = 30.0
CAR_LENGTH = 40
CAR_WIDTH = 20
CAR_RADIUS = 14                    # collision circle
MAX_SPEED = 140.0
MAX_REVERSE = 40.0
ACCEL = 160.0
BRAKE = 260.0
FRICTION = 70.0
MAX_STEER = math.radians(35)

# ---- Mission ----
START_POS = (80.0, 620.0)
START_HEADING = -math.pi / 2       # facing up the screen
GOAL_POS = (920.0, 80.0)
GOAL_RADIUS = 28
WALL_THICKNESS = 12

# ---- Perception ----
CELL = 10                          # occupancy grid cell size (pixels)
INFLATE_PX = 18                    # safety margin = car radius + buffer
CAMERA_NOISE = 3                   # simulated sensor noise (0 turns it off)
COLOR_TOL = 30                     # color-match tolerance for detection
MIN_AREA = 30                      # ignore blobs smaller than this (pixels)

RAY_COUNT = 9                      # number of range-sensor beams
RAY_FOV = math.radians(180)        # beams spread across the front 180 degrees
RAY_MAX = 160.0                    # maximum sensing range (pixels)
RAY_STEP = 2.0                     # ray-march step size


# ---- Navigation and control ----
PERCEPTION_EVERY = 3               # run camera + detector every N frames
PATH_SPACING = 10.0                # resample the planned path every N pixels
CRUISE_SPEED = 100.0
MIN_SPEED = 25.0
LOOKAHEAD_MIN = 25.0
LOOKAHEAD_MAX = 55.0
LOOKAHEAD_GAIN = 0.30              # lookahead grows with speed
GOAL_SLOW_GAIN = 0.8               # slow down when close to the goal
PID_KP, PID_KI, PID_KD = 0.05, 0.02, 0.0

SAFETY_CONE = math.radians(25)     # front beams used by the safety layer
SAFETY_STOP = 30.0                 # emergency zone (pixels from car center)
SAFETY_SLOW = 80.0                 # start slowing down here
SAFETY_HALT = 18.0                 # closer than this: full stop
CREEP_SPEED = 20.0

REPLAN_COOLDOWN = 0.6              # seconds between replans
PATH_CHECK_PX = 200.0              # how far ahead to check the path
MAX_RUN_TIME = 90.0                # give up after this many seconds

INFLATE_PX = 26                    # wider safety margin (overrides the value above)
STUCK_TIME = 1.0                   # seconds without moving before recovery starts
RECOVER_TIME = 0.8                 # seconds spent backing up