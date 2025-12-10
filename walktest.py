from pylx16a.lx16a import *
import time

# ================== 基本配置 ==================
PORT = "/dev/ttyUSB0"   # 串口按你实际情况改

ANGLE_MIN = 40
ANGLE_MAX = 200

# 总动作时间（秒）和插值步数：可以调来改变快慢/柔和程度
MOVE_DURATION = 1.5     # 从起始角度到目标角度用多少秒
MOVE_STEPS    = 60      # 插值多少步，越大越平滑、越小越干脆


# ========== 你可以在这里填“起始角度”和“目标角度” ==========
# 注意：这些都是舵机自己的角度（0~240），不是逻辑角

# ✅ 起始姿态（例如你量好的“趴着”/“当前”角度）
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

# ✅ 目标姿态（例如“站立”姿态，你可以随便改）
TARGET_POSE = {
    1: 110,   # 右上髋
    2: 170,   # 右上膝

    3: 110,   # 右下髋
    4: 170,   # 右下膝

    5: 110,   # 左下髋
    6: 170,   # 左下膝

    7: 108,   # 左上髋
    8: 178    # 左上膝
}


# ================== 函数部分 ==================

def init_servos():
    """初始化 1~8 号舵机，设置限位，返回 {id: servo 对象}"""
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
    """
    直接把所有电机一次性移动到 pose 指定的角度
    pose 是 {id: angle}
    """
    print("Applying pose:")
    for sid, angle in pose.items():
        print(f"  ID{sid} -> {angle}°")
        servos[sid].move(angle)
    time.sleep(wait_time)


def go_to_pose_smooth(servos, start_pose, target_pose,
                      duration=MOVE_DURATION, steps=MOVE_STEPS):
    """
    从 start_pose 平滑移动到 target_pose
    start_pose / target_pose 都是 {id: angle}
    """
    print(f"\nSmooth move: duration={duration}s, steps={steps}")
    for step in range(steps + 1):
        alpha = step / steps
        for sid in range(1, 9):
            a0 = start_pose[sid]
            a1 = target_pose[sid]
            a  = a0 + (a1 - a0) * alpha   # 线性插值
            servos[sid].move(a)
        time.sleep(duration / steps)


def main():
    servos = init_servos()

    # 1. 先把电机移动到“给定的起始角度”
    print("\nMove to INITIAL_POSE ...")
    apply_pose(servos, INITIAL_POSE, wait_time=1.0)

    # 2. 再从这个起始姿态平滑移动到目标姿态
    print("\nMove from INITIAL_POSE to TARGET_POSE ...")
    go_to_pose_smooth(servos, INITIAL_POSE, TARGET_POSE,
                      duration=MOVE_DURATION, steps=MOVE_STEPS)

    print("\nDone.")


if __name__ == "__main__":
    main()
