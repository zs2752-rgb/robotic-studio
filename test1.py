from pylx16a.lx16a import *
import time

# ========= 串口和舵机分布 =========
PORT = "/dev/ttyUSB0"   # 如果不是这个端口，改成实际端口

SERVO_MAP = {
    "RF_HIP": 1,  # 右前髋
    "RF_KNEE": 2, # 右前膝

    "RR_HIP": 3,  # 右后髋
    "RR_KNEE": 4, # 右后膝

    "LR_HIP": 5,  # 左后髋
    "LR_KNEE": 6, # 左后膝

    "LF_HIP": 7,  # 左前髋
    "LF_KNEE": 8  # 左前膝
}

# ==== 方向反的关节（你之前说 1、2、7、8）====
REVERSED_JOINTS = {
    "RF_HIP",   # ID 1
    "RF_KNEE",  # ID 2
    "LF_HIP",   # ID 7
    "LF_KNEE"   # ID 8
}

ANGLE_MIN = 40
ANGLE_MAX = 200
ANGLE_SUM = ANGLE_MIN + ANGLE_MAX   # = 240，用来做镜像

# ====== 1. 你要在这里填“初始姿态时的各电机硬件角度” ======
# 这些数字就是：机器人一开始你摆好姿态，然后每个舵机当前角度（0~240）；
# 先随便填一个占位，等你有真实数值再改。
HARDWARE_START_POSE = {
    "RF_HIP": 110,   # 这里改成你实际量到的角度
    "RF_KNEE": 100,
    "RR_HIP": 130,
    "RR_KNEE": 140,
    "LR_HIP": 148,
    "LR_KNEE": 150,
    "LF_HIP": 120,
    "LF_KNEE": 70
}

# ====== 2. 定义“站立姿态”的逻辑角度（可以慢慢调）======
NEUTRAL_ANGLE = 120

STAND_POSE = {
    "RF_HIP": NEUTRAL_ANGLE,        # 髋中立
    "RF_KNEE": NEUTRAL_ANGLE + 25,  # 膝稍微弯

    "RR_HIP": NEUTRAL_ANGLE,
    "RR_KNEE": NEUTRAL_ANGLE + 25,

    "LR_HIP": NEUTRAL_ANGLE,
    "LR_KNEE": NEUTRAL_ANGLE + 25,

    "LF_HIP": NEUTRAL_ANGLE,
    "LF_KNEE": NEUTRAL_ANGLE + 25,
}


# ---------- 角度转换工具函数 ----------

def hw_to_logical_angle(joint_name: str, hw_angle: float) -> float:
    """硬件角度 -> 逻辑角度（考虑方向反的舵机）"""
    if joint_name in REVERSED_JOINTS:
        return ANGLE_SUM - hw_angle
    else:
        return hw_angle

def logical_to_hw_angle(joint_name: str, logical_angle: float) -> float:
    """逻辑角度 -> 硬件角度"""
    if joint_name in REVERSED_JOINTS:
        return ANGLE_SUM - logical_angle
    else:
        return logical_angle


# ---------- 初始化 & 控制 ----------

def init_servos():
    """初始化串口和舵机，设置角度限位"""
    LX16A.initialize(PORT)

    servos = {}
    try:
        for name, sid in SERVO_MAP.items():
            s = LX16A(sid)
            s.set_angle_limits(ANGLE_MIN, ANGLE_MAX)
            servos[name] = s
            print(f"{name} (ID={sid}) init OK")
        time.sleep(0.5)
    except ServoTimeoutError as e:
        print(f"ERROR: servo ID {e.id_} not responding during init. Exit.")
        raise

    return servos

def move_joint_logical(servos, joint_name: str, logical_angle: float):
    """用逻辑角度移动单个关节"""
    hw = logical_to_hw_angle(joint_name, logical_angle)
    servos[joint_name].move(hw)

def go_to_pose_smooth(servos, start_pose, target_pose,
                      duration=3.0, steps=80):
    """从 start_pose 平滑插到 target_pose（都用逻辑角度）"""
    for step in range(steps + 1):
        alpha = step / steps
        for name in SERVO_MAP.keys():
            a0 = start_pose[name]
            a1 = target_pose[name]
            a = a0 + (a1 - a0) * alpha
            move_joint_logical(servos, name, a)
        time.sleep(duration / steps)


def stand_from_known_start(servos):
    """从你给定的初始硬件角度 -> 站立姿态"""

    # 1. 把你填的硬件角度转换成逻辑角度作为 start_pose
    logical_start = {}
    for name, hw_angle in HARDWARE_START_POSE.items():
        logical_angle = hw_to_logical_angle(name, hw_angle)
        logical_start[name] = logical_angle
        print(f"{name}: hw_start={hw_angle:.1f}°, logical_start={logical_angle:.1f}°")

    # 2. 平滑插值到 STAND_POSE
    print("\nStanding up from given initial pose...")
    go_to_pose_smooth(servos, logical_start, STAND_POSE,
                      duration=3.0, steps=80)
    print("Stand-up motion done.")


def main():
    servos = init_servos()
    stand_from_known_start(servos)


if __name__ == "__main__":
    main()
