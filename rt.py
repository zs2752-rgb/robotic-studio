import time
import math
import csv
from pylx16a.lx16a import *

PORT = "/dev/ttyUSB0"

# ---------------- 你的电机角度限位（按你实际改） ----------------
ANGLE_MIN = 40
ANGLE_MAX = 200

# ---------------- 站立姿态（你现在用的） ----------------
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

# ---------------- 腿与舵机映射 ----------------
# leg: hip_id, knee_id
LEG_MAP = {
    "RF": (1, 2),
    "RR": (3, 4),
    "LR": (5, 6),
    "LF": (7, 8),
}

# ---------------- 步态参数（你主要调这些） ----------------
HIP_AMP = 8.0        # 髋前后摆幅（越大步越大）
KNEE_LIFT = 14.0     # 抬腿时膝额外弯曲（越大抬得越高）
STEP_TIME = 0.02     # 每帧间隔（越小越快）
STEPS_PER_CYCLE = 50 # 每个周期的离散帧数（越大越平滑）
CYCLES = 6           # 走多少个周期

# 左右/前后平衡微调（解决走歪/打转）
# >1 代表更大幅度，<1 代表更小幅度
HIP_GAIN = {
    "RF": 1.0,
    "RR": 1.0,
    "LR": 1.0,
    "LF": 1.0,
}

# 膝盖也可以微调抬腿高度
KNEE_GAIN = {
    "RF": 1.0,
    "RR": 1.0,
    "LR": 1.0,
    "LF": 1.0,
}

# 对角组：trot
GROUP_A = ["LF", "RR"]
GROUP_B = ["RF", "LR"]

# ---------------- 工具函数 ----------------
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

def smooth_to_pose(servos, target_pose, duration=1.0, steps=50):
    start = read_pose(servos)
    for k in range(steps + 1):
        alpha = k / steps
        for sid in range(1, 9):
            a = start[sid] + (target_pose[sid] - start[sid]) * alpha
            a = clamp(servos, sid, a)
            servos[sid].move(a)
        time.sleep(duration / steps)

def send_leg_angles(servos, leg, hip_angle, knee_angle):
    hip_id, knee_id = LEG_MAP[leg]
    hip_angle = clamp(servos, hip_id, hip_angle)
    knee_angle = clamp(servos, knee_id, knee_angle)
    servos[hip_id].move(hip_angle)
    servos[knee_id].move(knee_angle)
    return hip_angle, knee_angle

# ---------------- 主步态：对角小跑（trot） ----------------
def trot_walk(servos, log_csv=False, csv_name="angle_log.csv"):
    hip0 = {leg: STAND_POSE[LEG_MAP[leg][0]] for leg in LEG_MAP}
    knee0 = {leg: STAND_POSE[LEG_MAP[leg][1]] for leg in LEG_MAP}

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

            # 每帧记录的角度（指令角）
            angles = {i: None for i in range(1, 9)}

            for leg in ["LF", "RR", "RF", "LR"]:
                phase = phi if leg in GROUP_A else (phi + math.pi)
                swing = math.sin(phase)  # -1~1

                # 髋：前后摆
                hip_angle = hip0[leg] + (HIP_AMP * HIP_GAIN[leg]) * swing

                # 膝：抬腿期（swing>0）更弯
                lift = (KNEE_LIFT * KNEE_GAIN[leg]) * max(0.0, swing)
                knee_angle = knee0[leg] + lift

                hip_cmd, knee_cmd = send_leg_angles(servos, leg, hip_angle, knee_angle)

                hip_id, knee_id = LEG_MAP[leg]
                angles[hip_id] = hip_cmd
                angles[knee_id] = knee_cmd

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

# ---------------- main ----------------
def main():
    servos = init_servos()

    # 直接站到 STAND_POSE（从当前姿态平滑过去）
    print("Go to STAND...")
    smooth_to_pose(servos, STAND_POSE, duration=1.2, steps=60)

    # 直接从站立开始走（可选记录到CSV）
    print("Walking...")
    trot_walk(servos, log_csv=True, csv_name="angle_log.csv")

    # 走完回到站立
    print("Back to STAND...")
    smooth_to_pose(servos, STAND_POSE, duration=1.0, steps=50)

    print("Done.")

if __name__ == "__main__":
    main()
