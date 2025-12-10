from pylx16a.lx16a import *
import time

# ========= 串口和舵机分布 =========
PORT = "/dev/ttyUSB0"   # 如果不是这个端口，改成实际端口

# 每条腿两个关节：HIP（髋）、KNEE（膝）
SERVO_MAP = {
    "RF_HIP": 1,  # Right Front hip
    "RF_KNEE": 2, # Right Front knee

    "RR_HIP": 3,  # Right Rear hip
    "RR_KNEE": 4, # Right Rear knee

    "LR_HIP": 5,  # Left Rear hip
    "LR_KNEE": 6, # Left Rear knee

    "LF_HIP": 7,  # Left Front hip
    "LF_KNEE": 8  # Left Front knee
}

# ==== 需要反向的关节 ====
# 你说 1、2、7、8 方向反了，所以把这四个 name 放进来
REVERSED_JOINTS = {
    "RF_HIP",   # ID 1
    "RF_KNEE",  # ID 2
    "LF_HIP",   # ID 7
    "LF_KNEE"   # ID 8
}

# 安全角度范围（你之前用的是 40~200）
ANGLE_MIN = 40
ANGLE_MAX = 200
ANGLE_SUM = ANGLE_MIN + ANGLE_MAX   # 用来做镜像，40 + 200 = 240

# 中立角度（逻辑空间里间接控制）
NEUTRAL_ANGLE = 120


def logical_to_hw_angle(joint_name: str, logical_angle: float) -> float:
    """
    把“逻辑角度”转换成实际要发给舵机的角度。
    对于方向反的关节，做一个镜像：angle_hw = ANGLE_SUM - logical_angle
    """
    if joint_name in REVERSED_JOINTS:
        return ANGLE_SUM - logical_angle
    else:
        return logical_angle


def init_servos():
    """初始化串口和所有舵机，设置角度限制，回中立位"""
    LX16A.initialize(PORT)

    servos = {}
    try:
        for name, sid in SERVO_MAP.items():
            s = LX16A(sid)
            s.set_angle_limits(ANGLE_MIN, ANGLE_MAX)
            servos[name] = s
            print(f"{name} (ID={sid}) init OK")
        time.sleep(0.5)

        # 先全部回到中立位（逻辑角度 120）
        set_all_to_neutral(servos)
        print("All servos moved to neutral (logical 120°).")
        time.sleep(1.0)

    except ServoTimeoutError as e:
        print(f"ERROR: servo ID {e.id_} not responding during init. Exit.")
        raise

    return servos


def move_joint(servos, joint_name: str, logical_angle: float):
    """
    移动单个关节（用逻辑角度控制，内部自动处理反向）
    """
    hw_angle = logical_to_hw_angle(joint_name, logical_angle)
    servos[joint_name].move(hw_angle)


def set_all_to_neutral(servos):
    """所有关节回到逻辑中立角度"""
    for name in SERVO_MAP.keys():
        move_joint(servos, name, NEUTRAL_ANGLE)
    time.sleep(0.8)


def set_leg_angles(servos, leg_prefix, hip_angle, knee_angle):
    """
    控制一条腿的两个关节（使用逻辑角度）。
    leg_prefix: 'RF' / 'RR' / 'LR' / 'LF'
    """
    hip_name = f"{leg_prefix}_HIP"
    knee_name = f"{leg_prefix}_KNEE"
    move_joint(servos, hip_name, hip_angle)
    move_joint(servos, knee_name, knee_angle)


def pose_stand(servos):
    """简单站立姿态（四条腿参数一致，逻辑空间）"""
    hip = NEUTRAL_ANGLE         # 髋部中立
    knee = NEUTRAL_ANGLE + 15   # 膝盖稍微弯一点
    for leg in ("RF", "RR", "LR", "LF"):
        set_leg_angles(servos, leg, hip, knee)
    time.sleep(0.8)


def pose_crouch(servos):
    """简单蹲下姿态（逻辑空间）"""
    hip = NEUTRAL_ANGLE - 10
    knee = NEUTRAL_ANGLE + 40
    for leg in ("RF", "RR", "LR", "LF"):
        set_leg_angles(servos, leg, hip, knee)
    time.sleep(0.8)


def lift_right_front_leg(servos):
    """抬右前腿测试动作（逻辑空间）"""
    # 其他三条腿先保持站立
    pose_stand(servos)

    # 右前腿：髋往上抬一点，膝更弯
    # 注意：这里的角度都是“逻辑角度”，反向舵机会在 logical_to_hw_angle 里被自动翻转
    hip_up = NEUTRAL_ANGLE - 25
    knee_bend = NEUTRAL_ANGLE + 40

    print("Lifting right front leg...")
    set_leg_angles(servos, "RF", hip_up, knee_bend)
    time.sleep(1.0)

    # 放回站立姿态
    print("Put right front leg back to stand pose...")
    pose_stand(servos)
    time.sleep(0.8)


def main():
    servos = init_servos()

    print("Pose: stand")
    pose_stand(servos)
    time.sleep(2.0)

    print("Pose: crouch")
    pose_crouch(servos)
    time.sleep(2.0)

    print("Pose: stand again")
    pose_stand(servos)
    time.sleep(2.0)

    # 抬右前腿测试
    lift_right_front_leg(servos)

    print("Back to neutral and finish.")
    set_all_to_neutral(servos)


if __name__ == "__main__":
    main()
