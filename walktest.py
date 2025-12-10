from pylx16a.lx16a import *
import time

PORT = "/dev/ttyUSB0"   # 按你的实际串口改

ANGLE_MIN = 40
ANGLE_MAX = 200

# ----------------- 1. 选定起始姿态（你自己填） -----------------
# 建议先填成你现在“趴着 / 初始”那一组角度
INITIAL_POSE = {
    1: 120,   # 右上髋
    2: 120,   # 右上膝
    3: 120,   # 右下髋
    4: 120,   # 右下膝
    5: 120,   # 左下髋
    6: 120,   # 左下膝
    7: 120,   # 左上髋
    8: 120    # 左上膝
}

# ----------------- 2. 站立姿态（基准站姿） -----------------
STAND_POSE = {
    1: 110,   # 右上髋
    2: 170,   # 右上膝

    3: 110,   # 右下髋
    4: 170,   # 右下膝

    5: 110,   # 左下髋
    6: 170,   # 左下膝

    7: 108,   # 左上髋（稍微更抬）
    8: 178    # 左上膝（稍微更弯）
}

# ----------------- 3. 步态调试参数（可以疯狂改） -----------------
# 抬腿时膝关节多弯多少（角度越大，抬得越高）
LIFT_KNEE_DELTA   = 10
# 抬腿时髋关节向前摆多少（正负看方向，如果走反了就改成负的）
LIFT_HIP_DELTA    = -5   # 如果发现它往后走，把这个改成 +5 再试

# “身体前移”时，支撑腿髋关节的相对变化
BODY_SHIFT_DELTA  = 4

# 一个小步的时间 & 插值步数
STEP_DURATION = 0.6
STEP_STEPS    = 30

# 一共走几个完整步态周期
NUM_CYCLES = 3


# ================== 基础函数 ==================

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


def apply_pose(servos, pose, wait_time=0.8):
    """一次性移动到某个姿态 pose（字典：{id: angle}）"""
    print("Apply pose:")
    for sid, angle in pose.items():
        print(f"  ID{sid} -> {angle}°")
        servos[sid].move(angle)
    time.sleep(wait_time)


def smooth_move(servos, start_pose, target_pose,
                duration=STEP_DURATION, steps=STEP_STEPS):
    """从 start_pose 平滑移动到 target_pose"""
    for step in range(steps + 1):
        alpha = step / steps
        for sid in range(1, 9):
            a0 = start_pose[sid]
            a1 = target_pose[sid]
            a  = a0 + (a1 - a0) * alpha
            servos[sid].move(a)
        time.sleep(duration / steps)


def clone_pose(pose):
    """浅拷贝一个姿态 dict"""
    return {sid: angle for sid, angle in pose.items()}


# ================== 步态生成 ==================

def make_stand_pose():
    """返回一份可修改的站立姿态副本"""
    return clone_pose(STAND_POSE)


def make_step_phases(base_pose):
    """
    基于站立姿态 base_pose，生成一个“往前走一步”的几个相位。
    这里用的是最简单的爬行步态，每个相位都是一个 {id: angle}。
    相位顺序：
      1. 抬左前（7,8）
      2. 左前落地，身体前移
      3. 抬右前（1,2）
      4. 右前落地，身体前移
      5. 抬左后（5,6）
      6. 左后落地，身体前移
      7. 抬右后（3,4）
      8. 右后落地，身体前移
    """
    phases = []
    pose = clone_pose(base_pose)

    # 为方便阅读，先展开一下变量
    # RF: 1,2    RR: 3,4    LR: 5,6    LF: 7,8

    # ----- 1. 抬左前腿 LF(7,8) -----
    p1 = clone_pose(pose)
    p1[7] = pose[7] + LIFT_HIP_DELTA
    p1[8] = pose[8] + LIFT_KNEE_DELTA
    phases.append(p1)

    # ----- 2. 左前落下到前方，同时其他腿髋略向后（身体向前） -----
    p2 = clone_pose(p1)
    # 左前落地，髋比原始站立略向前（再加一点）
    p2[7] = base_pose[7] + 2 * LIFT_HIP_DELTA
    p2[8] = base_pose[8]          # 膝回到站立角度

    # 其他三条腿髋向后一点（模拟身体向前）
    for hip_id in (1, 3, 5):
        p2[hip_id] = pose[hip_id] - BODY_SHIFT_DELTA
    phases.append(p2)

    pose = p2  # 更新当前姿态

    # ----- 3. 抬右前腿 RF(1,2) -----
    p3 = clone_pose(pose)
    p3[1] = pose[1] + LIFT_HIP_DELTA
    p3[2] = pose[2] + LIFT_KNEE_DELTA
    phases.append(p3)

    # ----- 4. 右前落下 + 身体前移 -----
    p4 = clone_pose(p3)
    p4[1] = base_pose[1] + 2 * LIFT_HIP_DELTA
    p4[2] = base_pose[2]

    for hip_id in (3, 5, 7):
        p4[hip_id] = pose[hip_id] - BODY_SHIFT_DELTA
    phases.append(p4)

    pose = p4

    # ----- 5. 抬左后腿 LR(5,6) -----
    p5 = clone_pose(pose)
    p5[5] = pose[5] + LIFT_HIP_DELTA
    p5[6] = pose[6] + LIFT_KNEE_DELTA
    phases.append(p5)

    # ----- 6. 左后落下 + 身体前移 -----
    p6 = clone_pose(p5)
    p6[5] = base_pose[5] + 2 * LIFT_HIP_DELTA
    p6[6] = base_pose[6]

    for hip_id in (1, 3, 7):
        p6[hip_id] = pose[hip_id] - BODY_SHIFT_DELTA
    phases.append(p6)

    pose = p6

    # ----- 7. 抬右后腿 RR(3,4) -----
    p7 = clone_pose(pose)
    p7[3] = pose[3] + LIFT_HIP_DELTA
    p7[4] = pose[4] + LIFT_KNEE_DELTA
    phases.append(p7)

    # ----- 8. 右后落下 + 身体前移 -----
    p8 = clone_pose(p7)
    p8[3] = base_pose[3] + 2 * LIFT_HIP_DELTA
    p8[4] = base_pose[4]

    for hip_id in (1, 5, 7):
        p8[hip_id] = pose[hip_id] - BODY_SHIFT_DELTA
    phases.append(p8)

    # 结束时的 pose，可以作为下一步的起点
    return phases, p8


# ================== 主流程 ==================

def main():
    servos = init_servos()

    # 1. 先回到你设定的起始姿态
    print("\nMove to INITIAL_POSE ...")
    apply_pose(servos, INITIAL_POSE, wait_time=1.0)

    # 2. 从起始姿态到站立姿态
    print("\nMove to STAND_POSE ...")
    smooth_move(servos, INITIAL_POSE, STAND_POSE,
                duration=1.5, steps=60)

    # 当前姿态设为站立
    current_pose = clone_pose(STAND_POSE)

    # 3. 往前走 NUM_CYCLES 个周期
    for cycle in range(NUM_CYCLES):
        print(f"\n=== Walk cycle {cycle+1}/{NUM_CYCLES} ===")
        phases, end_pose = make_step_phases(current_pose)
        for idx, phase_pose in enumerate(phases):
            print(f"  phase {idx+1}/{len(phases)}")
            smooth_move(servos, current_pose, phase_pose,
                        duration=STEP_DURATION, steps=STEP_STEPS)
            current_pose = clone_pose(phase_pose)
        # 更新基准
        current_pose = clone_pose(end_pose)

    print("\nDone walking.")


if __name__ == "__main__":
    main()
