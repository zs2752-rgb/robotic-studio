from pylx16a.lx16a import *
import time

PORT = "/dev/ttyUSB0"

ANGLE_MIN = 40
ANGLE_MAX = 200

# ----------------- 标准站立姿态 -----------------
# 可以用你现在觉得“最好看”的站立角度替换
STAND_POSE = {
    1: 130,  # 右上髋
    2: 60,  # 右上膝
    3: 100,  # 右下髋
    4: 180,  # 右下膝
    5: 130,  # 左下髋
    6: 180,  # 左下膝
    7: 100,  # 左上髋（稍微抬一点）
    8: 40,  # 左上膝（稍微多弯一点）
}

# ----------------- 步态参数（小步、防漂移用） -----------------
LIFT_KNEE_DELTA   =15    # 抬脚时膝多弯多少（越大抬得越高）
LIFT_HIP_DELTA    = +10   # 抬脚时髋往前摆多少；如果发现往后走就改成 +4
BODY_SHIFT_DELTA  = 12    # 身体前移量（越大实际前进越明显）

STEP_DURATION = 0.2      # 每个相位时间（秒）
STEP_STEPS    = 15       # 每个相位插值步数（越小越硬朗）

NUM_CYCLES    = 3        # 走几轮（每轮包含 8 个相位）


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
    """插值 + 角度夹紧，避免越界和抖动"""
    for step in range(steps + 1):
        alpha = step / steps
        for sid in range(1, 9):
            a0 = start_pose[sid]
            a1 = target_pose[sid]
            a  = a0 + (a1 - a0) * alpha

            # 把角度限制在舵机允许范围内
            min_ang, max_ang = servos[sid].get_angle_limits()
            if a < min_ang:
                a = min_ang
            if a > max_ang:
                a = max_ang

            servos[sid].move(a)

        time.sleep(duration / steps)


def clone_pose(pose):
    return {sid: angle for sid, angle in pose.items()}


# ========== 生成“一轮小步走”的相位 ==========

def make_step_phases(base_pose):
    """
    基于 base_pose（我们会传 STAND_POSE 进来），生成一轮 8 个相位。
    注意：只在函数内部用 base_pose 做参考，不会把偏移累加到下一轮。
    """
    phases = []
    pose = clone_pose(base_pose)

    # 约定：
    # RF: 1,2  右前
    # RR: 3,4  右后
    # LR: 5,6  左后
    # LF: 7,8  左前

    # ----- 1. 抬左前 LF(7,8) -----
    p1 = clone_pose(pose)
    p1[7] = pose[7] + LIFT_HIP_DELTA
    p1[8] = pose[8] + LIFT_KNEE_DELTA
    phases.append(p1)

    # ----- 2. 左前落地到前方 + 身体前移 -----
    p2 = clone_pose(p1)
    p2[7] = base_pose[7] + 2 * LIFT_HIP_DELTA
    p2[8] = base_pose[8]
    for hip in (1, 3, 5):  # 其它三条腿髋往后一点，模拟身体前移
        p2[hip] = pose[hip] - BODY_SHIFT_DELTA
    phases.append(p2)
    pose = p2

    # ----- 3. 抬右前 RF(1,2) -----
    p3 = clone_pose(pose)
    p3[1] = pose[1] + LIFT_HIP_DELTA
    p3[2] = pose[2] + LIFT_KNEE_DELTA
    phases.append(p3)

    # ----- 4. 右前落地 + 身体前移 -----
    p4 = clone_pose(p3)
    p4[1] = base_pose[1] + 2 * LIFT_HIP_DELTA
    p4[2] = base_pose[2]
    for hip in (3, 5, 7):
        p4[hip] = pose[hip] - BODY_SHIFT_DELTA
    phases.append(p4)
    pose = p4

    # ----- 5. 抬左后 LR(5,6) -----
    p5 = clone_pose(pose)
    p5[5] = pose[5] + LIFT_HIP_DELTA
    p5[6] = pose[6] + LIFT_KNEE_DELTA
    phases.append(p5)

    # ----- 6. 左后落地 + 身体前移 -----
    p6 = clone_pose(p5)
    p6[5] = base_pose[5] + 2 * LIFT_HIP_DELTA
    p6[6] = base_pose[6]
    for hip in (1, 3, 7):
        p6[hip] = pose[hip] - BODY_SHIFT_DELTA
    phases.append(p6)
    pose = p6

    # ----- 7. 抬右后 RR(3,4) -----
    p7 = clone_pose(pose)
    p7[3] = pose[3] + LIFT_HIP_DELTA
    p7[4] = pose[4] + LIFT_KNEE_DELTA
    phases.append(p7)

    # ----- 8. 右后落地 + 身体前移 -----
    p8 = clone_pose(p7)
    p8[3] = base_pose[3] + 2 * LIFT_HIP_DELTA
    p8[4] = base_pose[4]
    for hip in (1, 5, 7):
        p8[hip] = pose[hip] - BODY_SHIFT_DELTA
    phases.append(p8)

    # 注意：这里不再返回 end_pose，防止主循环累积偏移
    return phases


# ========== 主流程：先站起，再多轮“小步走”，每轮都回到 STAND_POSE ==========

def main():
    servos = init_servos()

    # 1. 从当前状态平滑站到 STAND_POSE
    print("\nMove to STAND_POSE ...")
    cur = read_current_pose(servos)
    smooth_move(servos, cur, STAND_POSE, duration=1.0, steps=40)
    cur = clone_pose(STAND_POSE)

    # 2. 走 NUM_CYCLES 轮，每轮都：
    #    STAND_POSE -> 8 个相位 -> 回到 STAND_POSE
    for cycle in range(NUM_CYCLES):
        print(f"\n=== Walk cycle {cycle+1}/{NUM_CYCLES} ===")

        phases = make_step_phases(STAND_POSE)  # 每次都用同一个 STAND_POSE

        # 从当前（站立）姿态走完一轮相位
        for idx, phase_pose in enumerate(phases):
            print(f"  phase {idx+1}/{len(phases)}")
            smooth_move(servos, cur, phase_pose,
                        duration=STEP_DURATION, steps=STEP_STEPS)
            cur = clone_pose(phase_pose)

        # 一轮结束后，强制回到标准站立，消掉累积误差
        print("  -> back to stand")
        smooth_move(servos, cur, STAND_POSE, duration=0.4, steps=24)
        cur = clone_pose(STAND_POSE)
        time.sleep(0.2)

    print("\nDone walking; final pose is STAND_POSE.")


if __name__ == "__main__":
    main()
