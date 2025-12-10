from pylx16a.lx16a import *
import time

PORT = "/dev/ttyUSB0"

# 右前 & 右后（你说这四个方向现在是对的）
RF_HIP  = 1
RF_KNEE = 2
RR_HIP  = 3
RR_KNEE = 4

# 左后 & 左前（我们将用 1–4 的镜像角度来控制）
LR_HIP  = 5
LR_KNEE = 6
LF_HIP  = 7
LF_KNEE = 8

ANGLE_MIN = 40
ANGLE_MAX = 200
ANGLE_SUM = ANGLE_MIN + ANGLE_MAX   # = 240，作为镜像的对称中心

def mirror_angle(angle: float) -> float:
    """
    把右侧关节的角度镜像到左侧。
    假设结构左右对称，且中间角在 120° 附近。
    """
    return ANGLE_SUM - angle


def init_servos():
    LX16A.initialize(PORT)
    servos = {}

    # 创建 1–8 号舵机对象并设置限位
    for sid in range(1, 9):
        s = LX16A(sid)
        s.set_angle_limits(ANGLE_MIN, ANGLE_MAX)
        servos[sid] = s
        print(f"Servo {sid} init OK")

    time.sleep(0.5)
    return servos


def move_right_and_mirror(servos,
                          rf_hip, rf_knee,
                          rr_hip, rr_knee):
    """
    只指定右前/右后的四个角度，
    左前/左后自动用镜像角度。
    所有角度都直接用“硬件角度”（0~240）。
    """

    # ---- 右侧：直接用给定的角度 ----
    servos[RF_HIP].move(rf_hip)
    servos[RF_KNEE].move(rf_knee)
    servos[RR_HIP].move(rr_hip)
    servos[RR_KNEE].move(rr_knee)

    # ---- 左侧：用右侧的镜像角度 ----
    servos[LF_HIP].move(mirror_angle(rf_hip))
    servos[LF_KNEE].move(mirror_angle(rf_knee))

    servos[LR_HIP].move(mirror_angle(rr_hip))
    servos[LR_KNEE].move(mirror_angle(rr_knee))


def stand_pose(servos):
    """
    让机器人站起来（偏高一点的站姿）。
    只设计右侧的站立角度，左侧自动镜像。
    """

    # 这里可以按你现在效果再调：
    rf_hip  = 105
    rf_knee = 170
    rr_hip  = 105
    rr_knee = 170

    print("Go to stand pose (right side angles):")
    print(f"  RF_HIP={rf_hip}, RF_KNEE={rf_knee}, "
          f"RR_HIP={rr_hip}, RR_KNEE={rr_knee}")

    move_right_and_mirror(servos, rf_hip, rf_knee, rr_hip, rr_knee)
    time.sleep(1.5)


def neutral_pose(servos):
    """
    简单的中立姿态，方便你从一个安全姿态开始。
    同样只控制右侧，左侧镜像。
    """
    rf_hip  = 120
    rf_knee = 120
    rr_hip  = 120
    rr_knee = 120

    print("Go to neutral pose")
    move_right_and_mirror(servos, rf_hip, rf_knee, rr_hip, rr_knee)
    time.sleep(1.5)


def main():
    servos = init_servos()

    # 先到一个中立姿态
    neutral_pose(servos)

    # 再站起来
    stand_pose(servos)

    print("Done.")


if __name__ == "__main__":
    main()
