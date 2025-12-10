from pylx16a.lx16a import *
import time

# ========= 串口和舵机分布 =========
PORT = "/dev/ttyUSB0"

SERVO_MAP = {
    "RF_HIP": 1,
    "RF_KNEE": 2,

    "RR_HIP": 3,
    "RR_KNEE": 4,

    "LR_HIP": 5,
    "LR_KNEE": 6,

    "LF_HIP": 7,
    "LF_KNEE": 8
}

# ========= 只反 1,2,8；5,6,7 改成不反 =========
REVERSED_JOINTS = {
    "RF_HIP", "RF_KNEE",   # 舵机 1、2 继续反向
    "LF_KNEE"              # 新增：舵机 8 反向
    # 5,6,7 不在这里，所以现在按“正常方向”运动
}

ANGLE_MIN = 40
ANGLE_MAX = 200
ANGLE_SUM = ANGLE_MIN + ANGLE_MAX  # 240

# ====== 这里填你实际测到的起始硬件角度 ======
HARDWARE_START_POSE = {
    "RF_HIP": 110,
    "RF_KNEE": 100,
    "RR_HIP": 130,
    "RR_KNEE": 140,
    "LR_HIP": 148,
    "LR_KNEE": 150,
    "LF_HIP": 120,
    "LF_KNEE": 70
}

# ====== 站立姿态（如果之前那版还不错就先用它）======
STAND_POSE = {
    "RF_HIP": 105,
    "RF_KNEE": 170,

    "RR_HIP": 105,
    "RR_KNEE": 170,

    "LR_HIP": 105,
    "LR_KNEE": 170,

    "LF_HIP": 105,
    "LF_KNEE": 170
}

def hw_to_logical_angle(joint_name, hw):
    if joint_name in REVERSED_JOINTS:
        return ANGLE_SUM - hw
    return hw

def logical_to_hw_angle(joint_name, logical):
    if joint_name in REVERSED_JOINTS:
        return ANGLE_SUM - logical
    return logical

def init_servos():
    LX16A.initialize(PORT)
    servos = {}
    for name, sid in SERVO_MAP.items():
        s = LX16A(sid)
        s.set_angle_limits(ANGLE_MIN, ANGLE_MAX)
        servos[name] = s
        print(f"{name} (ID={sid}) init OK")
    return servos

def move_joint(servos, name, logical_angle):
    hw_angle = logical_to_hw_angle(name, logical_angle)
    servos[name].move(hw_angle)

def go_to_pose_smooth(servos, start_pose, end_pose,
                      duration=3.0, steps=80):
    for step in range(steps + 1):
        alpha = step / steps
        for name in SERVO_MAP.keys():
            a0 = start_pose[name]
            a1 = end_pose[name]
            a = a0 + (a1 - a0) * alpha
            move_joint(servos, name, a)
        time.sleep(duration / steps)

def stand_from_known_start(servos):
    logical_start = {}
    for name, hw in HARDWARE_START_POSE.items():
        logical_start[name] = hw_to_logical_angle(name, hw)
        print(f"{name}: HW={hw}, LOGICAL={logical_start[name]}")

    print("\nStanding up...")
    go_to_pose_smooth(servos, logical_start, STAND_POSE)
    print("Done.")

def main():
    servos = init_servos()
    stand_from_known_start(servos)

if __name__ == "__main__":
    main()
