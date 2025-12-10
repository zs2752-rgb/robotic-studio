from pylx16a.lx16a import *
import time

PORT = "/dev/ttyUSB0"

# 电机 ID 布局（方便你以后看）：
# 右上：1,2   右下：3,4   左下：5,6   左上：7,8
HIP_IDS  = [1, 3, 5, 7]   # 各腿靠近身体的关节
KNEE_IDS = [2, 4, 6, 8]   # 各腿靠近地面的关节

ANGLE_MIN = 40
ANGLE_MAX = 200
ANGLE_SUM = ANGLE_MIN + ANGLE_MAX   # 240，中点 120

# 现在约定：1–4 已经方向正确
# 把 5,6,7,8 的方向在代码里翻转，让逻辑角度统一
REVERSED_IDS = {5, 6, 7, 8}


# ---------- 角度转换 ----------

def hw_to_logical(sid: int, hw_angle: float) -> float:
    """硬件读到的角度 -> 逻辑角度"""
    if sid in REVERSED_IDS:
        return ANGLE_SUM - hw_angle
    return hw_angle

def logical_to_hw(sid: int, logical_angle: float) -> float:
    """逻辑角度 -> 发给舵机的角度"""
    if sid in REVERSED_IDS:
        return ANGLE_SUM - logical_angle
    return logical_angle


# ---------- 初始化 ----------

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


# ---------- 姿态控制 ----------

def read_current_logical_pose(servos):
    """读取当前姿态（逻辑角度），返回 dict: {sid: logical_angle}"""
    pose = {}
    for sid, s in servos.items():
        hw = s.get_physical_angle()
        logical = hw_to_logical(sid, hw)
        pose[sid] = logical
        print(f"ID{sid}: hw={hw:.1f}°, logical={logical:.1f}°")
    return pose


def go_to_pose_smooth(servos, start_pose, target_pose,
                      duration=3.0, steps=80):
    """从 start_pose 平滑插值到 target_pose（逻辑角度）"""
    for step in range(steps + 1):
        alpha = step / steps
        for sid in range(1, 9):
            a0 = start_pose[sid]
            a1 = target_pose[sid]
            a  = a0 + (a1 - a0) * alpha   # 线性插值
            hw = logical_to_hw(sid, a)
            servos[sid].move(hw)
        time.sleep(duration / steps)


def make_stand_pose():
    """构造一个“站立姿态”的逻辑角度字典"""
    pose = {}

    # 站得比较高一点：髋稍微抬起，膝盖多弯一些
    STAND_HIP  = 105   # 髋关节逻辑角
    STAND_KNEE = 170   # 膝关节逻辑角

    for sid in HIP_IDS:
        pose[sid] = STAND_HIP
    for sid in KNEE_IDS:
        pose[sid] = STAND_KNEE

    return pose


def stand_up_from_current(servos):
    print("Reading current pose...")
    current_pose = read_current_logical_pose(servos)

    print("\nBuilding stand pose...")
    stand_pose = make_stand_pose()

    print("\nStanding up...")
    go_to_pose_smooth(servos, current_pose, stand_pose,
                      duration=3.0, steps=80)
    print("Stand up done.")


# ---------- main ----------

def main():
    servos = init_servos()
    stand_up_from_current(servos)


if __name__ == "__main__":
    main()
