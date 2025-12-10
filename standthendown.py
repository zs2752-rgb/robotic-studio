from pylx16a.lx16a import *
import time

PORT = "/dev/ttyUSB0"   # 按你的实际串口改

# 俯视布局：
# 右上：1,2    右下：3,4
# 左下：5,6    左上：7,8

ANGLE_MIN = 40
ANGLE_MAX = 200

# ===== 站立姿态（可以继续在这里微调） =====
STAND_POSE = {
    1: 110,   # 右上髋
    2: 170,   # 右上膝

    3: 110,   # 右下髋
    4: 170,   # 右下膝

    5: 110,   # 左下髋
    6: 170,   # 左下膝

    7: 108,   # 左上髋：稍微再抬一点
    8: 178    # 左上膝：稍微再多弯一点，让这条腿更有劲
}

# ===== 趴下 / 蹲低姿态（你可以理解为“趴下后的目标角度”） =====
# 这里我设成一个比较低、像“蹲趴”的姿态，比站立姿态更矮一些
DOWN_POSE = {
    1: 125,
    2: 150,

    3: 125,
    4: 150,

    5: 125,
    6: 150,

    7: 125,
    8: 150
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
                      duration=1.2, steps=50):
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
    """先预加载左上腿，再站起来"""
    # 1. 读取当前姿态（机器人一开始的趴姿）
    current_pose = read_current_pose(servos)

    # 2. 预加载左上腿（7、8）：先让这条腿“蹲好一点”
    print("\nPre-load left-front leg (7,8)...")
    preload_hip  = 105   # 左上髋预加载角度
    preload_knee = 185   # 左上膝预加载角度（比最终站立再弯一点）

    servos[7].move(preload_hip)
    servos[8].move(preload_knee)
    time.sleep(0.5)      # 给一点时间让电机到位、先用上力

    # 3. 再读一遍当前姿态，作为“站立插值”的起点
    print("\nPose after preload:")
    start_pose = read_current_pose(servos)

    # 4. 从预加载后的姿态，平滑站到 STAND_POSE
    print("\nStanding up with preload...")
    go_to_pose_smooth(servos, start_pose, STAND_POSE,
                      duration=1.2, steps=50)
    print("Stand up done.")


def go_down_from_stand(servos):
    """从站立姿态平滑趴下（蹲低）"""
    print("\nReading pose before going down...")
    start_pose = read_current_pose(servos)

    print("\nGoing down (to crouch/lie pose)...")
    go_to_pose_smooth(servos, start_pose, DOWN_POSE,
                      duration=1.2, steps=50)
    print("Down pose done.")


def main():
    servos = init_servos()

    # 先站起来
    stand_up_with_preload(servos)

    # 站住一会儿
    print("\nHold stand pose...")
    time.sleep(1.5)

    # 再趴下 / 蹲低
    go_down_from_stand(servos)


if __name__ == "__main__":
    main()

