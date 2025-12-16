import time
import math
import csv
from pylx16a.lx16a import *

PORT = "/dev/ttyUSB0"

# ----------------- 限位（按你实际） -----------------
ANGLE_MIN = 40
ANGLE_MAX = 200

# ----------------- 站立姿态（你的） -----------------
STAND_POSE = {
    1: 130,  # RF hip
    2: 60,   # RF knee
    3: 100,  # RR hip
    4: 180,  # RR knee
    5: 130,  # LR hip
    6: 180,  # LR knee
    7: 100,  # LF hip
    8: 5,   # LF knee
}

# ----------------- 腿映射 -----------------
LEG_MAP = {
    "RF": (1, 2),  # right-front
    "RR": (3, 4),  # right-rear
    "LR": (5, 6),  # left-rear
    "LF": (7, 8),  # left-front
}

# ----------------- 步态节奏 -----------------
STEP_TIME = 0.02
STEPS_PER_CYCLE = 50
CYCLES = 6

# ----------------- 你要调的核心：每个电机的摆幅/方向/偏置 -----------------
# AMP：摆幅大小（度）
# DIR：方向 +1 或 -1（反向就改成 -1）
# OFF：额外偏置（在 STAND_POSE 基础上再加/减一点）
SERVO_AMP = {  # 你可以随便改每个电机的幅度
    1: 8.0,   # hip
    2: 14.0,  # knee
    3: 8.0,
    4: 14.0,
    5: 8.0,
    6: 14.0,
    7: 8.0,
    8: 14.0,
}

SERVO_DIR = {  # 反向就把对应 id 改成 -1
    1: +1,
    2: +1,
    3: +1,
    4: -1,   # 很多情况下后腿膝可能要反
    5: +1,
    6: -1,
    7: +1,
    8: +1,
}

SERVO_OFF = {  # 每个电机单独偏置（调站姿/让它更吃力或更省力）
    1: 0.0,
    2: 0.0,
    3: 0.0,
    4: 0.0,
    5: 0.0,
    6: 0.0,
    7: 0.0,
    8: 0.0,
}

# ----------------- trot 对角组 -----------------
GROUP_A = ["LF", "RR"]
GROUP_B = ["RF", "LR"]

# ----------------- 工具函数 -----------------
def init_servos():
    LX16A.initialize(PORT)
    servos = {}
    for sid in range(1, 9):
        s = LX16A(sid)
        s.set_angle_limits(ANGLE_MIN, ANGLE_MAX)
        servos[sid] = s
        print(f"Servo {sid} OK")
    time.sleep(0.5)
    return servos

def clamp(servos, sid, a):
    lo, hi = servos[sid].get_angle_limits()
    if a < lo: a = lo
    if a > hi: a = hi
    return a

def read_pose(servos):
    pose = {}
    for sid in range(1, 9):
        pose[sid] = servos[sid].get_physical_angle()
    return pose

def smooth_to_pose(servos, target_pose, duration=1.0, steps=60):
    start = read_pose(servos)
    for k in range(steps + 1):
        alpha = k / steps
        for sid in range(1, 9):
            a = start[sid] + (target_pose[sid] - start[sid]) * alpha
            a = clamp(servos, sid, a)
            servos[sid].move(a)
        time.sleep(duration / steps)

# ----------------- 核心：每个电机单独控制幅度 -----------------
def trot_with_per_servo_amp(servos, log_csv=True, csv_name="angle_log.csv"):
    base = {sid: STAND_POSE[sid] + SERVO_OFF[sid] for sid in range(1, 9)}
    total_steps = CYCLES * STEPS_PER_CYCLE
    t0 = time.time()

    writer = None
    f = None
    if log_csv:
        f = open(csv_name, "w", newline="")
        writer = csv.writer(f)
        writer.writerow(["t","id1","id2","id3","id4","id5","id6","id7","id8"])

    try:
        for step in range(total_steps):
            phi = 2.0 * math.pi * (step / STEPS_PER_CYCLE)

            angles = {i: None for i in range(1, 9)}

            for leg in ["LF", "RR", "RF", "LR"]:
                phase = phi if leg in GROUP_A else (phi + math.pi)
                swing = math.sin(phase)  # -1~1

                hip_id, knee_id = LEG_MAP[leg]

                # hip：直接用正弦（前后摆）
                hip_angle = base[hip_id] + SERVO_DIR[hip_id] * SERVO_AMP[hip_id] * swing

                # knee：只在抬腿期变化（swing>0），更稳
                lift = max(0.0, swing)
                knee_angle = base[knee_id] + SERVO_DIR[knee_id] * SERVO_AMP[knee_id] * lift

                hip_angle = clamp(servos, hip_id, hip_angle)
                knee_angle = clamp(servos, knee_id, knee_angle)

                servos[hip_id].move(hip_angle)
                servos[knee_id].move(knee_angle)

                angles[hip_id] = hip_angle
                angles[knee_id] = knee_angle

            if writer:
                writer.writerow([
                    time.time() - t0,
                    angles[1], angles[2], angles[3], angles[4],
                    angles[5], angles[6], angles[7], angles[8]
                ])

            time.sleep(STEP_TIME)

    finally:
        if f:
            f.close()

def main():
    servos = init_servos()

    # 先站好（注意加了 SERVO_OFF 后的站姿）
    stand_with_off = {sid: STAND_POSE[sid] + SERVO_OFF[sid] for sid in range(1, 9)}
    print("Go to STAND...")
    smooth_to_pose(servos, stand_with_off, duration=1.2, steps=70)

    print("Start walking (per-servo tunable)...")
    trot_with_per_servo_amp(servos, log_csv=True, csv_name="angle_log.csv")

    print("Back to STAND...")
    smooth_to_pose(servos, stand_with_off, duration=1.0, steps=60)

    print("Done.")

if __name__ == "__main__":
    main()
