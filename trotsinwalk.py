from pylx16a.lx16a import *
import time
import math

PORT = "/dev/ttyUSB0"

ANGLE_MIN = 40
ANGLE_MAX = 200

# ---------------- 站立姿态（基准） ----------------
# 可以用你现在调好的站立姿态替换
STAND_POSE = {
    1: 130,  # 右前髋 RF hip
    2: 60,  # 右前膝 RF knee
    3: 100,  # 右后髋 RR hip
    4: 180,  # 右后膝 RR knee
    5: 130,  # 左后髋 LR hip
    6: 180,  # 左后膝 LR knee
    7: 100,  # 左前髋 LF hip
    8: 40,  # 左前膝 LF knee
}

# -------- 步态参数（你主要调这里） --------
HIP_AMP      = 15    # 髋关节前后摆幅度（越大步越大，走反了就改成负号）
KNEE_LIFT    = 20.0   # 抬腿时膝盖额外弯曲的幅度
CYCLES       = 12      # 走多少个完整周期（一个周期=左前+右后 / 右前+左后 各走一次）
STEPS_PER_CYCLE = 15  # 每个周期离散多少帧（越大越平滑）

STEP_TIME    = 0.03   # 每一帧间隔时间（秒），越小越快

# 对角腿分组：
# RF:1,2   RR:3,4   LR:5,6   LF:7,8
GROUP_A = ["LF", "RR"]  # 一组：左前 + 右后
GROUP_B = ["RF", "LR"]  # 一组：右前 + 左后


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
    """用于从当前姿态平滑站到 STAND_POSE，一次性插值"""
    for step in range(steps + 1):
        alpha = step / steps
        for sid in range(1, 9):
            a0 = start_pose[sid]
            a1 = target_pose[sid]
            a  = a0 + (a1 - a0) * alpha

            # 夹在舵机自身限位内
            min_ang, max_ang = servos[sid].get_angle_limits()
            if a < min_ang:
                a = min_ang
            if a > max_ang:
                a = max_ang

            servos[sid].move(a)

        time.sleep(duration / steps)


def clamp_angle(servos, sid, angle):
    """把 angle 夹在舵机允许的范围内"""
    min_ang, max_ang = servos[sid].get_angle_limits()
    if angle < min_ang:
        angle = min_ang
    if angle > max_ang:
        angle = max_ang
    return angle


def trot_sine_walk(servos):
    """
    正弦对角小跑：
      - GROUP_A (LF + RR) 使用相位 phi
      - GROUP_B (RF + LR) 使用相位 phi + pi
    每条腿：
      hip = hip0 + HIP_AMP * sin(phase)
      knee = knee0 + max(0, KNEE_LIFT * swing)   # swing>0 时抬腿弯膝
    """

    # 方便写：把 STAND_POSE 拆成各腿基准
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

    # 把“腿名字”映射到舵机 ID
    hip_id = {"RF": 1, "RR": 3, "LR": 5, "LF": 7}
    knee_id = {"RF": 2, "RR": 4, "LR": 6, "LF": 8}

    total_steps = CYCLES * STEPS_PER_CYCLE

    for step in range(total_steps):
        # phi 在 [0, 2*pi*CYCLES] 区间均匀扫描
        phi = 2.0 * math.pi * (step / STEPS_PER_CYCLE)

        # Group A 使用相位 phi，Group B 使用 phi + pi（反相）
        legs = ["LF", "RR", "RF", "LR"]
        for leg in legs:
            if leg in GROUP_A:
                phase = phi
            else:
                phase = phi + math.pi

            swing = math.sin(phase)  # -1 ~ 1

            # 髋关节：围绕基准左右摆
            hip_angle = hip0[leg] + HIP_AMP * swing

            # 膝关节：swing>0 时表示“向前摆/腾空期”，膝盖加弯
            lift = KNEE_LIFT * max(0.0, swing)
            knee_angle = knee0[leg] + lift

            # 发送到舵机（注意夹限位）
            sid_hip = hip_id[leg]
            sid_knee = knee_id[leg]

            hip_angle  = clamp_angle(servos, sid_hip, hip_angle)
            knee_angle = clamp_angle(servos, sid_knee, knee_angle)

            servos[sid_hip].move(hip_angle)
            servos[sid_knee].move(knee_angle)

        time.sleep(STEP_TIME)


def main():
    servos = init_servos()

    # 1. 从当前随便什么姿态，平滑站到 STAND_POSE
    print("\nMove to STAND_POSE ...")
    cur = read_current_pose(servos)
    smooth_move(servos, cur, STAND_POSE, duration=1.0, steps=40)

    # 2. 执行正弦对角小跑
    print("\nStart trot_sine_walk ...")
    trot_sine_walk(servos)

    # 3. 走完再回到 STAND_POSE
    print("\nBack to STAND_POSE ...")
    now_pose = read_current_pose(servos)
    smooth_move(servos, now_pose, STAND_POSE, duration=1.0, steps=40)

    print("\nDone.")


if __name__ == "__main__":
    main()
