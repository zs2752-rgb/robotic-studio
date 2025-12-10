from pylx16a.lx16a import *
import time

PORT = "/dev/ttyUSB0"

ANGLE_MIN = 40
ANGLE_MAX = 200

# -------- 站立姿态（根据你当前比较稳的站姿来） --------
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

# -------- 步态参数 --------
# 抬脚高度
LIFT_KNEE_DELTA = 15
# 髋关节前后摆动幅度（正负决定方向；如果整体往后走就反号）
HIP_SWING_DELTA = +35

# 左右修正系数：如果发现总是向一边转，可以微调
LEFT_GAIN  = 1.0   # 作用在左边两条腿（5,6,7,8）
RIGHT_GAIN = 1.0   # 作用在右边两条腿（1,2,3,4）

STEP_DURATION = 0.2   # 每个相位时间
STEP_STEPS    = 15    # 每个相位插值步数
NUM_CYCLES    = 10     # 走几轮（每轮两步：对角1 + 对角2）


# ========== 基础函数 ==========

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
                duration=STEP_DURATION, steps=STEP_STEPS):
    for step in range(steps + 1):
        alpha = step / steps
        for sid in range(1, 9):
            a0 = start_pose[sid]
            a1 = target_pose[sid]
            a  = a0 + (a1 - a0) * alpha

            # 夹在舵机限位内
            min_ang, max_ang = servos[sid].get_angle_limits()
            if a < min_ang:
                a = min_ang
            if a > max_ang:
                a = max_ang

            servos[sid].move(a)
        time.sleep(duration / steps)


def clone_pose(pose):
    return {sid: angle for sid, angle in pose.items()}


# ========== 对角小跑步态 ==========

def make_trot_phases(base_pose):
    """
    对角 gait：
      相位1：抬 LF(7,8) + RR(3,4)，向前摆
      相位2：落 LF+RR 回到基准
      相位3：抬 RF(1,2) + LR(5,6)，向前摆
      相位4：落 RF+LR 回到基准
    这里 base_pose 就用 STAND_POSE，每轮都从它出发，不累积偏移。
    """
    phases = []

    # ---- phase 1: 抬 LF + RR ----
    p1 = clone_pose(base_pose)

    # 左前 LF：7/8（乘以 LEFT_GAIN）
    p1[7] = base_pose[7] + HIP_SWING_DELTA * LEFT_GAIN
    p1[8] = base_pose[8] + LIFT_KNEE_DELTA * LEFT_GAIN

    # 右后 RR：3/4（乘以 RIGHT_GAIN）
    p1[3] = base_pose[3] + HIP_SWING_DELTA * RIGHT_GAIN
    p1[4] = base_pose[4] + LIFT_KNEE_DELTA * RIGHT_GAIN

    phases.append(p1)

    # ---- phase 2: LF + RR 落地回到站立 ----
    p2 = clone_pose(base_pose)
    phases.append(p2)

    # ---- phase 3: 抬 RF + LR ----
    p3 = clone_pose(base_pose)

    # 右前 RF：1/2
    p3[1] = base_pose[1] + HIP_SWING_DELTA * RIGHT_GAIN
    p3[2] = base_pose[2] + LIFT_KNEE_DELTA * RIGHT_GAIN

    # 左后 LR：5/6
    p3[5] = base_pose[5] + HIP_SWING_DELTA * LEFT_GAIN
    p3[6] = base_pose[6] + LIFT_KNEE_DELTA * LEFT_GAIN

    phases.append(p3)

    # ---- phase 4: RF + LR 落地回到站立 ----
    p4 = clone_pose(base_pose)
    phases.append(p4)

    return phases


# ========== 主流程 ==========

def main():
    servos = init_servos()

    # 先站到 STAND_POSE
    print("\nMove to STAND_POSE ...")
    cur = read_current_pose(servos)
    smooth_move(servos, cur, STAND_POSE, duration=1.0, steps=40)
    cur = clone_pose(STAND_POSE)

    # 多轮对角小跑
    for cycle in range(NUM_CYCLES):
        print(f"\n=== Trot cycle {cycle+1}/{NUM_CYCLES} ===")
        phases = make_trot_phases(STAND_POSE)  # 每轮都从 STAND_POSE 定义

        for idx, phase in enumerate(phases):
            print(f"  phase {idx+1}/{len(phases)}")
            smooth_move(servos, cur, phase,
                        duration=STEP_DURATION, steps=STEP_STEPS)
            cur = clone_pose(phase)

        # 每轮结束回到 STAND_POSE（消掉误差）
        smooth_move(servos, cur, STAND_POSE, duration=0.3, steps=20)
        cur = clone_pose(STAND_POSE)
        time.sleep(0.1)

    print("\nDone; final pose = STAND_POSE.")


if __name__ == "__main__":
    main()
