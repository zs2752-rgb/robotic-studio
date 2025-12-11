from pylx16a.lx16a import *
import time
import math

PORT = "/dev/ttyUSB0"

ANGLE_MIN = 40
ANGLE_MAX = 200

# ---------------- 你的 STAND_POSE ----------------
STAND_POSE = {
    1: 130,  # 右前髋 RF hip
    2: 60,   # 右前膝 RF knee
    3: 100,  # 右后髋 RR hip
    4: 180,  # 右后膝 RR knee
    5: 130,  # 左后髋 LR hip
    6: 180,  # 左后膝 LR knee
    7: 100,  # 左前髋 LF hip
    8: 5,   # 左前膝 LF knee
}

# -------- 步态参数 --------
HIP_AMP   = 18.0    # 基本髋摆幅（所有腿的基础振幅）
KNEE_LIFT = 20.0   # 抬腿时膝盖额外弯曲

CYCLES          = 8      # 走多少个完整周期
STEPS_PER_CYCLE = 25     # 每周期多少帧
STEP_TIME       = 0.03   # 每帧间隔时间

# 对角腿分组
GROUP_A = ["LF", "RR"]   # 左前 + 右后
GROUP_B = ["RF", "LR"]   # 右前 + 左后

# -------- 关键：每条腿单独的髋关节增益 --------
# 如果你觉得左前hip不怎么动，就把 "LF" 的值设大一点，比如 1.8 或 2.0
HIP_GAIN = {
    "RF": 1.0,
    "RR": 1.0,
    "LR": 1.0,
    "LF": 1.5,   # 左前髋放大 1.8 倍
}

# 如有需要，膝盖也可以做类似增益
KNEE_GAIN = {
    "RF": 1.0,
    "RR": 1.0,
    "LR": 1.0,
    "LF": 1.0,
}


def init_servos():
    LX16A.initialize(PORT)
    servos = {}
    for sid in range(1, 9):
        s = LX16A(sid)
        s.set_angle_limits(ANGLE_MIN, ANGLE_MAX)
        servos[sid] = s
        print(f"Servo {sid} init OK")
    time.sleep(0.5)
    return servos


def read_current_pose(servos):
    pose = {}
    print("Current pose:")
    for sid, s in servos.items():
        a = s.get_physical_angle()
        pose[sid] = a
        print(f"  ID{sid}: {a:.1f}°")
    return pose


def smooth_move(servos, start_pose, target_pose,
                duration=1.0, steps=40):
    for step in range(steps + 1):
        alpha = step / steps
        for sid in range(1, 9):
            a0 = start_pose[sid]
            a1 = target_pose[sid]
            a  = a0 + (a1 - a0) * alpha

            min_ang, max_ang = servos[sid].get_angle_limits()
            if a < min_ang:
                a = min_ang
            if a > max_ang:
                a = max_ang

            servos[sid].move(a)
        time.sleep(duration / steps)


def clamp_angle(servos, sid, angle):
    min_ang, max_ang = servos[sid].get_angle_limits()
    if angle < min_ang:
        angle = min_ang
    if angle > max_ang:
        angle = max_ang
    return angle


def trot_sine_walk(servos):
    # 基准角
    hip0 = {
        "RF": STAND_POSE[1],
        "RR": STAND_POSE[3],
        "LR": STAND_POSE[5],
        "LF": STAND_POSE[7],
    }
    knee0 = {
        "RF": STAND_POSE[2],
        "RR": STAND_POSE[4],
        "LR": STAND_POSE[6],
        "LF": STAND_POSE[8],
    }

    hip_id  = {"RF": 1, "RR": 3, "LR": 5, "LF": 7}
    knee_id = {"RF": 2, "RR": 4, "LR": 6, "LF": 8}

    total_steps = CYCLES * STEPS_PER_CYCLE

    for step in range(total_steps):
        phi = 2.0 * math.pi * (step / STEPS_PER_CYCLE)

        for leg in ["LF", "RR", "RF", "LR"]:
            if leg in GROUP_A:
                phase = phi
            else:
                phase = phi + math.pi

            swing = math.sin(phase)  # -1 ~ 1

            # 髋关节：加上单独的 HIP_GAIN[leg]
            hip_angle = hip0[leg] + HIP_AMP * HIP_GAIN[leg] * swing

            # 膝关节：抬腿期 swing>0 时弯曲，并带 KNEE_GAIN
            lift = KNEE_LIFT * KNEE_GAIN[leg] * max(0.0, swing)
            knee_angle = knee0[leg] + lift

            sid_hip  = hip_id[leg]
            sid_knee = knee_id[leg]

            hip_angle  = clamp_angle(servos, sid_hip,  hip_angle)
            knee_angle = clamp_angle(servos, sid_knee, knee_angle)

            servos[sid_hip].move(hip_angle)
            servos[sid_knee].move(knee_angle)

        time.sleep(STEP_TIME)


def main():
    servos = init_servos()

    print("\nMove to STAND_POSE ...")
    cur = read_current_pose(servos)
    smooth_move(servos, cur, STAND_POSE, duration=1.0, steps=40)

    print("\nStart trot_sine_walk ...")
    trot_sine_walk(servos)

    print("\nBack to STAND_POSE ...")
    now_pose = read_current_pose(servos)
    smooth_move(servos, now_pose, STAND_POSE, duration=1.0, steps=40)

    print("\nDone.")


if __name__ == "__main__":
    main()
