from pylx16a.lx16a import *
import time

PORT = "/dev/ttyUSB0"   # 串口按你之前用的来

# 机身俯视：
# 右上：1,2    右下：3,4
# 左下：5,6    左上：7,8

ANGLE_MIN = 40
ANGLE_MAX = 200

# ===== 在这里设定你想要的“站立姿态”的角度（舵机自己的角度） =====
# 先给一个大概的示例，你可以一边试一边改：
STAND_POSE = {
    1: 130,   # 右上髋
    2: 60,   # 右上膝

    3: 110,   # 右下髋
    4: 185,   # 右下膝

    5: 170,   # 左下髋
    6: 185,   # 左下膝

    7: 100,   # 左上髋
    8: 40.09    # 左上膝
}


def init_servos():
    """初始化 1~8 号舵机"""
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


def go_to_pose_smooth(servos, start_pose, target_pose,
                      duration=1, steps=10):
    """从 start_pose 平滑移动到 target_pose"""
    for step in range(steps + 1):
        alpha = step / steps
        for sid in range(1, 9):
            a0 = start_pose[sid]
            a1 = target_pose[sid]
            a  = a0 + (a1 - a0) * alpha    # 线性插值
            servos[sid].move(a)
        time.sleep(duration / steps)


def stand_up(servos):
    # 1. 读当前姿态作为起点（避免突然跳）
    current_pose = read_current_pose(servos)

    # 2. 目标就是 STAND_POSE
    target_pose = STAND_POSE

    print("\nStanding up...")
    go_to_pose_smooth(servos, current_pose, target_pose,
                      duration=3.0, steps=80)
    print("Done.")


def main():
    servos = init_servos()
    stand_up(servos)


if __name__ == "__main__":
    main()
