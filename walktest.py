from pylx16a.lx16a import *
import time

PORT = "/dev/ttyUSB0"   # 按你的实际串口改

ANGLE_MIN = 40
ANGLE_MAX = 200

# ----------------- 站立姿态（基准站姿） -----------------
# 可以继续根据你现在的“最好看站姿”来改
STAND_POSE = {
    1: 110,   # 右上髋
    2: 170,   # 右上膝

    3: 110,   # 右下髋
    4: 170,   # 右下膝

    5: 110,   # 左下髋
    6: 170,   # 左下膝

    7: 108,   # 左上髋（稍微再抬一点）
    8: 178    # 左上膝（稍微再多弯一点）
}

# ----------------- 步态调试参数（可以根据感觉疯狂改） -----------------
LIFT_KNEE_DELTA   = 10   # 抬腿时膝关节多弯多少
LIFT_HIP_DELTA    = -5   # 抬腿时髋向前摆多少（走反了就改成 +5）
BODY_SHIFT_DELTA  = 4    # 身体“往前”时，支撑腿髋关节的改变量

STEP_DURATION = 0.6      # 每个小相位用时（秒）
STEP_STEPS    = 30       # 每个小相位插值步数（越大越柔和）

NUM_CYCLES    = 3        # 一共走几轮完整步态


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


def read_current_pose(servos):
    """读取当前角度，返回 {id: angle}"""
    pose = {}
    print("Current pose:")
    for sid, s in servos.items():
        a = s.get_physical_angle()
        pose[sid] = a
        print(f"  ID{sid}: {a:.1f}°")
    return pose


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
    return {sid: angle for sid, angle in pose.items()}


# ================== 步态生成 ==================

def make_step_phases(base_pose):
    """
    基于站立姿态 base_pose，生成一个“往前走一轮”的相位列表。
    简单爬行步态，相位顺序：
      1. 抬左前 (7,8)
      2. 左前落地 + 身体前移
      3. 抬右前 (1,2)
      4. 右前落地 + 身体前移
      5. 抬左后 (5,6)
      6. 左后落地 + 身体前移
      7. 抬右后 (3,4)
      8. 右后落地 + 身体前移
    返回：phases, end_pose
      phases: [pose1, pose2, ...]
      end_pose: 最后一个相位的姿态，可作为下一轮的起点
    """
    phases = []
    pose = clone_pose(base_pose)

    # RF: 1,2    RR: 3,4    LR: 5,6    LF: 7,8

    # ----- 1. 抬左前腿 LF(7,8) -----
    p1 = clone_pose(pose)
    p1[7] = pose[7] + LIFT_HIP_DELTA
    p1[8] = pose[8] + LIFT_KNEE_DELTA
    phases.append(p1)

    # ----- 2. 左前落下到前方 + 身体前移 -----
    p2 = clone_pose(p1)
    # 左前髋再比原站立略“更向前”
    p2[7] = base_pose[7] + 2 * LIFT_HIP_DELTA
    p2[8] = base_pose[8]          # 膝回到站立角度

    # 其他三条腿的髋向后一点，模拟躯干前移
    for hip_id in (1, 3, 5):
        p2[hip_id] = pose[hip_id] - BODY_SHIFT_DELTA
    phases.append(p2)

    pose = p2

    # ----- 3. 抬右前 RF(1,2) -----
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

    # ----- 5. 抬左后 LR(5,6) -----
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

    # ----- 7. 抬右后 RR(3,4) -----
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

    end_pose = p8
    return phases, end_pose


# ================== 主流程 ==================

def main():
    servos = init_servos()

    # 1. 从当前角度平滑到 STAND_POSE（不再需要用户给初始姿态）
    print("\nRead current pose and move to STAND_POSE ...")
    current_pose = read_current_pose(servos)
    smooth_move(servos, current_pose, STAND_POSE,
                duration=1.5, steps=60)

    # 当前姿态设为站立
    current_pose = clone_pose(STAND_POSE)

    # 2. 往前走 NUM_CYCLES 个周期
    for cycle in range(NUM_CYCLES):
        print(f"\n=== Walk cycle {cycle+1}/{NUM_CYCLES} ===")
        phases, end_pose = make_step_phases(current_pose)

        for idx, phase_pose in enumerate(phases):
            print(f"  phase {idx+1}/{len(phases)}")
            smooth_move(servos, current_pose, phase_pose,
                        duration=STEP_DURATION, steps=STEP_STEPS)
            current_pose = clone_pose(phase_pose)

        current_pose = clone_pose(end_pose)

    print("\nDone walking.")


if __name__ == "__main__":
    main()
