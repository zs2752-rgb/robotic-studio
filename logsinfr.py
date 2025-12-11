import time
import math
import csv
from pylx16a.lx16a import *

PORT = "/dev/ttyUSB0"

# ----------------- 你的 STAND POSE -----------------
STAND_POSE = {
    1: 130,  # RF hip
    2: 60,   # RF knee
    3: 100,  # RR hip
    4: 180,  # RR knee
    5: 130,  # LR hip
    6: 180,  # LR knee
    7: 130,  # LF hip
    8: 60,   # LF knee
}

# ---------------- 步态参数 ----------------
HIP_AMP   = 20
KNEE_LIFT = 15

CYCLES = 3
STEPS_PER_CYCLE = 20
STEP_TIME = 0.03

# 对角腿组
GROUP_A = ["LF", "RR"]
GROUP_B = ["RF", "LR"]

# 关节 ID 映射
hip_id  = {"RF": 1, "RR": 3, "LR": 5, "LF": 7}
knee_id = {"RF": 2, "RR": 4, "LR": 6, "LF": 8}


def clamp_angle(servos, sid, angle):
    min_ang, max_ang = servos[sid].get_angle_limits()
    return max(min_ang, min(max_ang, angle))


def init_servos():
    LX16A.initialize(PORT)
    servos = {}
    for sid in range(1, 9):
        s = LX16A(sid)
        s.set_angle_limits(40, 200)
        servos[sid] = s
        print(f"Servo {sid} OK")
    time.sleep(1)
    return servos


def move_to_stand(servos, duration=1.0, steps=40):
    print("Moving to stand pose...")
    for step in range(steps + 1):
        a = step / steps
        for sid in range(1, 9):
            target = STAND_POSE[sid]
            servos[sid].move(target)
        time.sleep(duration / steps)
    time.sleep(0.3)


# ------------------ ⭐ 带日志记录的步态函数 -------------------

def trot_sine_walk_with_log(servos, logfile="angle_log.csv"):
    print(f"Logging angles to {logfile}")

    # 提取基准角
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

    total_steps = CYCLES * STEPS_PER_CYCLE
    t0 = time.time()

    # 打开 CSV 日志文件
    with open(logfile, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "id1","id2","id3","id4","id5","id6","id7","id8"])

        for step in range(total_steps):
            phi = 2 * math.pi * (step / STEPS_PER_CYCLE)

            angles = {i: None for i in range(1, 9)}

            for leg in ["LF", "RR", "RF", "LR"]:

                # 相位切换
                if leg in GROUP_A:
                    phase = phi
                else:
                    phase = phi + math.pi

                swing = math.sin(phase)

                hip_angle = hip0[leg] + HIP_AMP * swing
                lift = KNEE_LIFT * max(0.0, swing)
                knee_angle = knee0[leg] + lift

                sid_hip  = hip_id[leg]
                sid_knee = knee_id[leg]

                hip_angle  = clamp_angle(servos, sid_hip,  hip_angle)
                knee_angle = clamp_angle(servos, sid_knee, knee_angle)

                servos[sid_hip].move(hip_angle)
                servos[sid_knee].move(knee_angle)

                angles[sid_hip]  = hip_angle
                angles[sid_knee] = knee_angle

            # 时间戳
            tnow = time.time() - t0

            writer.writerow([
                tnow,
                angles[1], angles[2], angles[3], angles[4],
                angles[5], angles[6], angles[7], angles[8]
            ])

            time.sleep(STEP_TIME)

    print("Done logging.")


# ------------------ 主程序 ---------------------

def main():
    servos = init_servos()
    move_to_stand(servos)
    trot_sine_walk_with_log(servos, logfile="angle_log.csv")
    move_to_stand(servos)
    print("Finished.")


if __name__ == "__main__":
    main()
