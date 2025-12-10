from pylx16a.lx16a import *
import time

# ========= 串口和舵机分布 =========
PORT = "/dev/ttyUSB0"   # 如果你的串口不是这个，请修改

SERVO_MAP = {
    "RF_HIP": 1,  # 右前髋关节
    "RF_KNEE": 2, # 右前膝关节

    "RR_HIP": 3,  # 右后髋
    "RR_KNEE": 4, # 右后膝

    "LR_HIP": 5,  # 左后髋
    "LR_KNEE": 6, # 左后膝

    "LF_HIP": 7,  # 左前髋
    "LF_KNEE": 8  # 左前膝
}

# ==== 需要反向的关节（你之前说 1、2、7、8 方向反了） ====
REVERSED_JOINTS = {
    "RF_HIP",   # ID 1
    "RF_KNEE",  # ID 2
    "LF_HIP",   # ID 7
    "LF_KNEE"   # ID 8
}

ANGLE_MIN = 40
ANGLE_MAX = 200
ANGLE_SUM = ANGLE_MIN + ANGLE_MAX  # = 240，用于反向计算

NEUTRAL_ANGLE = 120  # 逻辑上的“中立角度”

def logical_to_hw_angle(joint_name: str, logical_angle: float) -> float:
    """逻辑角度 → 硬件舵机角度，如果关节反向则做镜像映射。"""
    if joint_name in REVERSED_JOINTS:
        return ANGLE_SUM - logical_angle
    else:
        return logical_angle

def init_servos():
    """初始化串口和所有舵机，设置角度限位。"""
    LX16A.initialize(PORT)

    servos = {}
    try:
        for name, sid in SERVO_MAP.items():
            s = LX16A(sid)
            s.set_angle_limits(ANGLE_MIN, ANGLE_MAX)
            servos[name] = s
            print(f"{name} (ID={sid}) initialized")
        time.sleep(0.5)
    except ServoTimeoutError as e:
        print(f"ERROR: servo ID {e.id_} not responding during init. Exiting.")
        return None

    return servos

def move_joint_logical(servos, joint_name: str, logical_angle: float):
    """按照逻辑角度移动某个关节（自动处理反向）。"""
    hw = logical_to_hw_angle(joint_name, logical_angle)
    servos[joint_name].move(hw)

def set_all_to_neutral(servos):
    """所有舵机回到逻辑中立角度。"""
    for name in SERVO_MAP.keys():
        move_joint_logical(servos, name, NEUTRAL_ANGLE)
    time.sleep(1.0)

def stand_pose(servos):
    """设定一个基本站立姿态（逻辑角度）。"""
    # 这个姿态你可以根据实际结构微调
    hip = NEUTRAL_ANGLE
    knee = NEUTRAL_ANGLE + 25
    for leg in ("RF", "RR", "LR", "LF"):
        move_joint_logical(servos, f"{leg}_HIP", hip)
        move_joint_logical(servos, f"{leg}_KNEE", knee)
    time.sleep(1.0)

def main():
    servos = init_servos()
    if servos is None:
        return

    print("Moving all servos to neutral position...")
    set_all_to_neutral(servos)

    print("Moving to stand pose...")
    stand_pose(servos)

    print("Done.")

if __name__ == "__main__":
    main()
