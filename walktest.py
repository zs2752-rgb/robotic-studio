from pylx16a.lx16a import *
import time

PORT = "/dev/ttyUSB0"

ANGLE_MIN = 40
ANGLE_MAX = 200

# ---------- 站立姿态 ----------
STAND_POSE = {
    1: 130,
    2: 60,
    3: 100,
    4: 180,
    5: 130,
    6: 180,
    7: 100,
    8: 40,
}

# ---------- 步态参数（已经调快一点） ----------
LIFT_KNEE_DELTA   = 20    # 抬脚高度
LIFT_HIP_DELTA    = +8    # 髋向前摆的量；如果发现往后走就改成 +6
BODY_SHIFT_DELTA  = 12     # 身体前移量（越大迈步越大）

STEP_DURATION = 0.2       # 每个相位 0.3 秒（比之前快一倍）
STEP_STEPS    = 15        # 每个相位 18 步（更干脆）

NUM_CYCLES    = 3         # 走几轮


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
    """插值 + 角度 clamp，避免越界报错"""
    for step in range(steps + 1):
        alpha = step / steps
        for sid in range(1, 9):
            a0 = start_pose[sid]
            a1 = target_pose[sid]
            a  = a0 + (a1 - a0) * alpha

            # 重要：把角度夹在舵机限位内
            min_ang, max_ang = servos[sid].get_angle_limits()
            if a < min_ang:
                a = min_ang
            if a > max_ang:
                a = max_ang

            servos[sid].move(a)

        time.sleep(duration / steps)


def clone_pose(pose):
    return {sid: angle for sid, angle in pose.items()}


# ========== 步态相位 ==========

def make_step_phases(base_pose):
    phases = []
    pose = clone_pose(base_pose)

    # RF:1,2  RR:3,4  LR:5,6  LF:7,8

    # 1 抬左前 LF
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

    # 3 抬右前 RF
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

    # 5 抬左后 LR
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

    # 7 抬右后 RR
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

    end_pose = p8
    return phases, end_pose


# ========== 主流程 ==========

def main():
    servos = init_servos()

    # 1. 从当前姿态 → 站立
    print("\nMove to STAND_POSE ...")
    current = read_current_pose(servos)
    smooth_move(servos, current, STAND_POSE,
                duration=1.0, steps=40)
    current = clone_pose(STAND_POSE)

    # 2. 走路
    for cycle in range(NUM_CYCLES):
        print(f"\n=== Walk cycle {cycle+1}/{NUM_CYCLES} ===")
        phases, end_pose = make_step_phases(current)
        for phase in phases:
            smooth_move(servos, current, phase)
            current = clone_pose(phase)
        current = clone_pose(end_pose)

    # 3. 走完以后，再回到标准站立姿态
    print("\nBack to STAND_POSE ...")
    smooth_move(servos, current, STAND_POSE,
                duration=1.0, steps=40)

    print("\nDone.")


if __name__ == "__main__":
    main()
