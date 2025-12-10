from pylx16a.lx16a import *
import time

PORT = "/dev/ttyUSB0"   # 按你实际的串口来改

# 俯视布局：
# 右上：1,2    右下：3,4
# 左下：5,6    左上：7,8

ANGLE_MIN = 40
ANGLE_MAX = 200

# ===== 站立姿态（舵机自己的角度，可以再慢慢调） =====
# 这里给左上腿（7、8）稍微多一点弯曲，让它更有劲
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
                      duration=1, steps=20):
    """
    从 start_pose 平滑移动到 target_pose
    duration 越小、steps 越少 -> 动作越快、越有劲
    """
    for step in range(steps + 1):
        alpha = step / steps
        for sid in range(1, 9):
            a0 = start_pose[sid]
            a1 = target_pose[sid]
            a  = a0 + (a1 - a0) * alpha    # 线性插值
            servos[sid].move(a)
        time.sleep(duration / steps)


def stand_up_with_preload(servos):
    # 1. 先读取当前姿态，方便你看初始角
    current_pose = read_current_pose(servos)

    # 2. 预加载左上腿（7、8）：先让这条腿先“蹲好一点”
    print("\nPre-load left-front leg (7,8)...")
    preload_hip  = 105   # 左上髋预加载角度
    preload_knee = 185   # 左上膝预加载角度（比最终站立再弯一点）

    servos[7].move(preload_hip)
    servos[8].move(preload_knee)
    time.sleep(0.5)      # 给一点时间让电机到位、用上力

    # 3. 再读一遍当前姿态，作为平滑插值的起点
    print("\nPose after preload:")
    start_pose = read_current_pose(servos)

    # 4. 从预加载后的姿态，平滑站到 STAND_POSE
    print("\nStanding up with preload...")
    go_to_pose_smooth(servos, start_pose, STAND_POSE,
                      duration=1.2, steps=50)
    print("Stand up done.")


def main():
    servos = init_servos()
    stand_up_with_preload(servos)


if __name__ == "__main__":
    main()

