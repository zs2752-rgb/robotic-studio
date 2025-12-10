from pylx16a.lx16a import *
import time

PORT = "/dev/ttyUSB0"

ANGLE_MIN = 40
ANGLE_MAX = 200

# ---------- 1. 对称的基础站立姿态（不要加补偿） ----------
BASE_STAND_POSE = {
    1: 130,  # 右前髋 RF hip
    2: 60,  # 右前膝 RF knee
    3: 100,  # 右后髋 RR hip
    4: 180,  # 右后膝 RR knee
    5: 130,  # 左后髋 LR hip
    6: 180,  # 左后膝 LR knee
    7: 100,  # 左前髋 LF hip
    8: 40,  # 左前膝 LF knee
}

# ---------- 2. 身体倾斜修正参数（重点！） ----------
# ROLL_ADJ > 0 : 左边腿“变长一点”、右边“变短一点” → 把身体往右扶正
# ROLL_ADJ < 0 : 反过来
ROLL_ADJ = 5.0   # 先试 3 度，不够再慢慢加到 4、5

def build_stand_pose():
    """在基础姿态上加左右补偿，得到真正用的 STAND_POSE"""
    pose = dict(BASE_STAND_POSE)

    # 左边腿：5,6,7,8  —— 让它们“长一点”（hip 小一点，knee 大一点）
    pose[5] -= ROLL_ADJ   # 左后髋
    pose[6] += ROLL_ADJ   # 左后膝
    pose[7] -= ROLL_ADJ   # 左前髋
    pose[8] += ROLL_ADJ   # 左前膝

    # 右边腿：1,2,3,4  —— 让它们“短一点”（hip 大一点，knee 小一点）
    pose[1] += ROLL_ADJ   # 右前髋
    pose[2] -= ROLL_ADJ   # 右前膝
    pose[3] += ROLL_ADJ   # 右后髋
    pose[4] -= ROLL_ADJ   # 右后膝

    return pose


# ---------- 3. 步态参数（小步、防漂移） ----------
LIFT_KNEE_DELTA   = 15    # 抬脚时膝多弯多少
LIFT_HIP_DELTA    = -6   # 抬脚时髋往前摆多少（反向走就改成 +4）
BODY_SHIFT_DELTA  = 20    # 身体前移量

STEP_DURATION = 0.2      # 每个相位时间
STEP_STEPS    = 15       # 每个相位插值步数

NUM_CYCLES    = 3        # 走几轮


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

            # 夹在舵机自身限位内
            min_ang, max_ang = servos[sid].get_angle_limits()
            if a < min_ang:
                a = min_ang
            if a > max_ang:
                a = max_ang

            servos[sid].move(a)

        time.sleep(duration / steps)


def clone_pose(pose):
    return {sid: angle for sid, angle in pose.items()}


# ========== 一轮“小步走”的相位（不累积偏移） ==========

def make_step_phases(base_pose):
    """
    base_pose 就是已经补偿过的 STAND_POSE。
    一轮 8 个相位，走完后再回到 base_pose，防止越走越歪。
    """
    phases = []
    pose = clone_pose(base_pose)

    # RF:1,2  RR:3,4  LR:5,6  LF:7,8

    # 1 抬左前 LF(7,8)
    p1 = clone_pose(pose)
    p1[7] = pose[7] + LIFT_HIP_DELTA
    p1[8] = pose[8] + LIFT_KNEE_DELTA
    phases.append(p1)

    # 2 左前落地 + 身体前移
    p2 = clone_pose(p1)
    p2[7] = base_pose[7] + 2 * LIFT_HIP_DELTA
    p2[8] = base_pose[8]
    for hip in (1, 3, 5):
        p2[hip] = pose[hip] - BODY_SHIFT_DELTA
    phases.append(p2)
    pose = p2

    # 3 抬右前 RF(1,2)
    p3 = clone_pose(pose)
    p3[1] = pose[1] + LIFT_HIP_DELTA
    p3[2] = pose[2] + LIFT_KNEE_DELTA
    phases.append(p3)

    # 4 右前落地 + 身体前移
    p4 = clone_pose(p3)
    p4[1] = base_pose[1] + 2 * LIFT_HIP_DELTA
    p4[2] = base_pose[2]
    for hip in (3, 5, 7):
        p4[hip] = pose[hip] - BODY_SHIFT_DELTA
    phases.append(p4)
    pose = p4

    # 5 抬左后 LR(5,6)
    p5 = clone_pose(pose)
    p5[5] = pose[5] + LIFT_HIP_DELTA
    p5[6] = pose[6] + LIFT_KNEE_DELTA
    phases.append(p5)

    # 6 左后落地 + 身体前移
    p6 = clone_pose(p5)
    p6[5] = base_pose[5] + 2 * LIFT_HIP_DELTA
    p6[6] = base_pose[6]
    for hip in (1, 3, 7):
        p6[hip] = pose[hip] - BODY_SHIFT_DELTA
    phases.append(p6)
    pose = p6

    # 7 抬右后 RR(3,4)
    p7 = clone_pose(pose)
    p7[3] = pose[3] + LIFT_HIP_DELTA
    p7[4] = pose[4] + LIFT_KNEE_DELTA
    phases.append(p7)

    # 8 右后落地 + 身体前移
    p8 = clone_pose(p7)
    p8[3] = base_pose[3] + 2 * LIFT_HIP_DELTA
    p8[4] = base_pose[4]
    for hip in (1, 5, 7):
        p8[hip] = pose[hip] - BODY_SHIFT_DELTA
    phases.append(p8)

    return phases


# ========== 主流程 ==========

def main():
    servos = init_servos()

    # 根据 ROLL_ADJ 生成带补偿的 STAND_POSE
    stand_pose = build_stand_pose()

    # 1. 从当前姿态平滑站到 stand_pose
    print("\nMove to stand_pose ...")
    cur = read_current_pose(servos)
    smooth_move(servos, cur, stand_pose, duration=1.0, steps=40)
    cur = clone_pose(stand_pose)

    # 2. 走 NUM_CYCLES 轮，每轮都：
    #    stand_pose -> 8 个相位 -> 回到 stand_pose
    for cycle in range(NUM_CYCLES):
        print(f"\n=== Walk cycle {cycle+1}/{NUM_CYCLES} ===")

        phases = make_step_phases(stand_pose)

        for idx, phase in enumerate(phases):
            print(f"  phase {idx+1}/{len(phases)}")
            smooth_move(servos, cur, phase,
                        duration=STEP_DURATION, steps=STEP_STEPS)
            cur = clone_pose(phase)

        # 回到修正后的站立姿态，消掉累积误差
        print("  -> back to stand_pose")
        smooth_move(servos, cur, stand_pose, duration=0.4, steps=24)
        cur = clone_pose(stand_pose)
        time.sleep(0.2)

    print("\nDone; final pose is stand_pose.")


if __name__ == "__main__":
    main()
