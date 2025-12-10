from pylx16a.lx16a import *
import time

# ========= 串口和舵机分布 =========
PORT = "/dev/ttyUSB0"   # 如果你的串口不是这个，改成实际的

# 每条腿两个关节：HIP（靠近身体）、KNEE（靠近脚）
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

# 为了安全，先用比较保守的角度范围
ANGLE_MIN = 40
ANGLE_MAX = 200

# 中立角度（你可以根据实际零位再慢慢调）
NEUTRAL_ANGLE = 120

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

        # 先全部回到中立位
        set_all_to_neutral(servos)
        print("All servos moved to neutral.")
        time.sleep(1.0)

    except ServoTimeoutError as e:
        print(f"ERROR: servo ID {e.id_} not responding during init. Exit.")
        raise

    return servos


def set_all_to_neutral(servos):
    """所有关节到中立角度"""
    for name, s in servos.items():
        s.move(NEUTRAL_ANGLE)
    # 给一点时间慢慢到位
    time.sleep(0.8)


def set_leg_angles(servos, leg_prefix, hip_angle, knee_angle):
    """
    控制一条腿的两个关节。
    leg_prefix: 'RF' / 'RR' / 'LR' / 'LF'
    """
    hip_name = f"{leg_prefix}_HIP"
    knee_name = f"{leg_prefix}_KNEE"
    servos[hip_name].move(hip_angle)
    servos[knee_name].move(knee_angle)


def pose_stand(servos):
    """简单站立姿态（四条腿参数一致）"""
    hip = NEUTRAL_ANGLE      # 髋部大致在中立
    knee = NEUTRAL_ANGLE + 15  # 膝盖稍微弯一点增加支撑
    for leg in ("RF", "RR", "LR", "LF"):
        set_leg_angles(servos, leg, hip, knee)
    time.sleep(0.8)


def pose_crouch(servos):
    """简单蹲下姿态"""
    hip = NEUTRAL_ANGLE - 10
    knee = NEUTRAL_ANGLE + 40
    for leg in ("RF", "RR", "LR", "LF"):
        set_leg_angles(servos, leg, hip, knee)
    time.sleep(0.8)


def lift_right_front_leg(servos):
    """抬右前腿做一个测试动作"""
    # 先保持其他三条腿在站姿
    pose_stand(servos)

    # 右前腿：髋往上抬一点，膝盖再弯一点
    hip_up = NEUTRAL_ANGLE - 25   # 数值要根据你实际关节方向微调
    knee_bend = NEUTRAL_ANGLE + 40

    print("Lifting right front leg...")
    set_leg_angles(servos, "RF", hip_up, knee_bend)
    time.sleep(1.0)

    # 再放回站立姿态
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
