"""场景几何（严格按规格推导）。坐标系: 原点=终点侧墙×图纸底边墙内角,
x:0→3.020 指向起点侧, y:0→2.000; yaw=0 车头朝 -x, 逆时针为正。"""
import math

FIELD_X, FIELD_Y = 3.020, 2.000
START_ZONE = (2.420, 3.020, 0.600, 1.400)
START_POSE = (2.720, 1.000, 0.0)        # 规格: x,y=yaw 与"车头朝 -x"配套
# 机器人模型自身 +x 是车头(相机在 +x), 而规格 yaw=0 表示车头朝 -x;
# 因此模型/驱动实际使用的 yaw = π。位置与朝向的物理含义与规格完全一致, 只是模型系差 180°。
MODEL_YAW = math.pi
END_ZONE   = (0.000, 0.800, 0.300, 1.700)

ROBOT_L = 0.24          # 车长(x, 沿行驶方向)
ROBOT_W = 0.16          # 车宽(y) —— 0.24x0.16, 必须 < 货架宽 0.380 才能钻入
END_SCAN = (END_ZONE[1] + ROBOT_L/2 + 0.01, (END_ZONE[2]+END_ZONE[3])/2)
START_SCAN = (START_ZONE[0] + ROBOT_L/2 + 0.01, (START_ZONE[2]+START_ZONE[3])/2)
RACK_D  = 0.170         # 货架 x 向深度
RACK_L  = 0.380         # 货架 y 向长度

# ---------------- 动态膨胀: 随"是否载货"切换 ----------------
def _circ(l, w): return math.hypot(l/2.0, w/2.0)
INFLATE       = _circ(ROBOT_L, ROBOT_W)
CARRY_D, CARRY_L = RACK_D, RACK_L        # 载货时货架占位(x,y)
INFLATE_EMPTY = _circ(ROBOT_L, ROBOT_W)                     # 0.1442
INFLATE_CARRY = _circ(max(ROBOT_L, RACK_D), RACK_L)

# Shared physical geometry and control limits.
ROBOT_H = 0.07
LIDAR_Z = 0.15
RACK_H = 0.2325
TOP_T = 0.015
LEG_T = 0.03
PLATFORM_TOP = 0.18
PLATFORM_T = 0.02
PLATFORM_L, PLATFORM_W = 0.20, 0.14
PLATFORM_MASS = 0.5
LIFT_P_GAIN = 100.0
LIFT_D_GAIN = 2 * math.sqrt(LIFT_P_GAIN * PLATFORM_MASS)
LIFT_DOWN, LIFT_UP, LIFT_UPPER = 0.0, 0.30, 0.40
PAYLOAD_BOTTOM = RACK_H - TOP_T
CONTACT_LIFT = PAYLOAD_BOTTOM - PLATFORM_TOP
LIFT_TOL = 0.008
EXIT_DIST = 0.45
CAM_POS = (0.11, 0.0, 0.32)
CAM_PITCH = -0.15
QR_SIZE, QR_Z = 0.12, RACK_H + 0.10
COLLISION_MARGIN = 0.050  # Extra clearance outside the car/payload union.
TRACKING_TOL = 0.008  # Per-axis endpoint error reserved by the planner.
CONTROL_SPEED = 0.30             # translation: previous 0.15 x 2
CONTROL_GAIN = 4.0               # near-goal speed scales by the same factor
TURN_SPEED = 0.375               # empty scan rotation: previous 0.25 x 1.5
TURN_GAIN = 2.25                 # near-heading speed: previous 1.5 x 1.5
TURN_COMMAND_LIMIT = 0.45        # driver limit: previous 0.30 x 1.5
ODOM_FRAME, MAP_FRAME, BASE_FRAME = "odom", "map", "base_link"
LASER_FRAME = "laser"
SLAM_MAP_FRAME = "slam_map"


def inflation_radius(hx,hy):
    """Inflate beyond the union footprint with clearance and tracking allowance."""
    return math.hypot(hx+COLLISION_MARGIN,hy+COLLISION_MARGIN)+TRACKING_TOL


def model_start_pose():
    return START_POSE[0], START_POSE[1], MODEL_YAW


def rack_model_center(n):
    x0, x1, y0, y1 = RACKS[n][:4]
    return (x0 + x1) / 2, (y0 + y1) / 2


def rack_leg_rects(n, center=None):
    x0, x1, y0, y1 = RACKS[n][:4]
    old_x, old_y = rack_model_center(n)
    cx, cy = center or (old_x, old_y)
    dx, dy = cx - old_x, cy - old_y
    return [(old_x - (x1-x0)*.45 + dx, old_x + (x1-x0)*.45 + dx,
             y0 + dy, y0 + LEG_T + dy),
            (old_x - (x1-x0)*.45 + dx, old_x + (x1-x0)*.45 + dx,
             y1 - LEG_T + dy, y1 + dy)]

def inflate_for(carrying):
    """按状态返回膨胀半径: 载货=车与货架并集的外接半径, 空车=车体外接半径。"""
    return INFLATE_CARRY if carrying else INFLATE

def footprint(carrying):
    """当前占位尺寸 (长x, 宽y)。"""
    return (max(ROBOT_L, CARRY_D), CARRY_L) if carrying else (ROBOT_L, ROBOT_W)

# Reflect rack layout left/right about the robot's initial forward line y=1.0.
# x remains unchanged; y_min/y_max/service_y map to 2-y_max/2-y_min/2-service_y.
# Framework identities viewed from spawn: A left/far, B right/near, C right/far, D left/near.
# name: (x_min, x_max, y_min, y_max, service_y, qr)
RACKS = {
 "A": (1.051, 1.221, 0.351, 0.729, 0.5400, "RACKA_XXXX"),
 "B": (1.756, 1.920, 1.189, 1.570, 1.3795, "RACKB_XXXX"),
 "C": (1.252, 1.422, 1.489, 1.870, 1.7200, "RACKC_XXXX"),
 "D": (1.853, 2.020, 0.201, 0.579, 0.3900, "RACKD_XXXX"),
}
# From the spawn viewpoint: inner left/right, then outer left/right.
DROP = {"A": (0.250,0.420), "B": (0.250,1.050), "C": (0.550,0.700), "D": (0.550,1.420)}
ORDER = ["A","B","C","D"]  # Framework task identity order.


AP_OFF, SCAN_OFF, RACK_OFF, EXIT_OFF = 0.45, 0.32, 0.00, 0.16
D_IN, D_OUT = 0.405, 0.245      # 规格给定; 与上式自洽(见 validate)

def face_x(n):   return RACKS[n][1]                    # 架面 = x_max
def ap(n):       return (face_x(n)+AP_OFF,   RACKS[n][4])
def scan(n):     return (face_x(n)+SCAN_OFF, RACKS[n][4])
def rack_ctr(n): return ((RACKS[n][0]+RACKS[n][1])/2, RACKS[n][4])
def exit_pt(n):
    """退出点: 举升后沿行进方向(-x)继续走 d_out, 停在货架另一侧(规格 d_out≈0.245)。
    注: 该点也等于 另一侧架面(x_min) 前 EXIT_OFF=0.16, 与规格'退出点(架面前0.16)'一致。"""
    return (rack_ctr(n)[0] - D_OUT, RACKS[n][4])

def rack_rect(n):
    x0,x1,y0,y1 = RACKS[n][:4]
    return (x0,x1,y0,y1)

def inflate_rect(r, m=None):
    m = INFLATE if m is None else m
    return (r[0]-m, r[1]+m, r[2]-m, r[3]+m)

def rect(n):     return RACKS[n][:4]

# ---------------- 运行时障碍地图: 货架被搬走后位置会变 ----------------
# 值 = (x0,x1,y0,y1) 或 None(已被搬走/在车上)。free() 以此为准, 而不是初始位置。
RACK_NOW = {}

def reset_rack_pos():
    """复位: 所有货架回到初始位置。"""
    RACK_NOW.clear()
    for n in RACKS: RACK_NOW[n] = rack_rect(n)

def set_rack_pos(n, center=None):
    """更新货架位置: center=(x,y) 放到该处; None 表示已不在原地(被举起/在车上)。"""
    if center is None:
        RACK_NOW[n] = None
    else:
        d, l = RACK_D/2, RACK_L/2
        RACK_NOW[n] = (center[0]-d, center[0]+d, center[1]-l, center[1]+l)

def rack_now(n):
    return RACK_NOW.get(n)

reset_rack_pos()

# ---------------- 激光雷达实测障碍(由 lidar_map 注入) ----------------
LIDAR_CHECK = None          # callable(x, y) -> True 表示该点被雷达实测占据

def set_lidar_check(fn):
    global LIDAR_CHECK
    LIDAR_CHECK = fn

def free(x, y, removed=(), infl=None):
    """点(x,y)是否可通行: 不出界且不进入未搬走货架的膨胀区。
    infl=None 用空车膨胀; 载货路段应显式传入 INFLATE_CARRY。"""
    m = INFLATE if infl is None else infl
    if LIDAR_CHECK is not None and LIDAR_CHECK(x, y, m):
        return False                       # 雷达实测占据
    if not (m <= x <= FIELD_X-m): return False
    if not (m <= y <= FIELD_Y-m): return False
    for n in RACKS:
        if n in removed: continue
        current = rack_now(n)
        if current is None: continue
        r = inflate_rect(current, m)
        if r[0] <= x <= r[1] and r[2] <= y <= r[3]: return False
    return True

def free_under(x, y, own, removed=()):
    """钻到 own 货架正下方时, 自身货架不参与碰撞(其余规则同 free)。"""
    if not (INFLATE <= x <= FIELD_X-INFLATE): return False
    if not (INFLATE <= y <= FIELD_Y-INFLATE): return False
    for n in RACKS:
        if n in removed or n == own: continue
        current = rack_now(n)
        if current is None: continue
        r = inflate_rect(current)
        if r[0] <= x <= r[1] and r[2] <= y <= r[3]: return False
    return True

def under_ok(n, x, y):
    """车体能否从 n 号货架下方通过: 宽度需小于货架开口, 且对准货架中心线。"""
    x0, x1, y0, y1 = rack_rect(n)
    return (ROBOT_W < RACK_L) and (x0 - 0.02 <= x <= x1 + 0.02) and abs(y - RACKS[n][4]) <= 0.05

def seg_hits_rack(p, q, removed=()):
    """轴对齐线段 p->q 是否穿过未搬走货架的膨胀区。"""
    (x0,y0),(x1,y1) = p,q
    steps = max(int(abs(x1-x0)/0.005), int(abs(y1-y0)/0.005), 1)
    for i in range(steps+1):
        t = i/steps
        if not free(x0+(x1-x0)*t, y0+(y1-y0)*t, removed): return True
    return False
