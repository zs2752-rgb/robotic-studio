from pylx16a.lx16a import *
import time

PORT = "/dev/ttyUSB0"   # 如果你的端口不是这个，请修改
SERVO_ID_1 = 1          # 第一个舵机 ID
SERVO_ID_2 = 2          # 第二个舵机 ID

ANGLE_MIN = 40
ANGLE_MAX = 200

def main():
    # 初始化串口
    LX16A.initialize(PORT)

    # 初始化两个舵机对象（不会立即与舵机通讯）
    servo1 = LX16A(SERVO_ID_1)
    servo2 = LX16A(SERVO_ID_2)

    # 设置角度范围（这里才会真正通讯）
    try:
        servo1.set_angle_limits(ANGLE_MIN, ANGLE_MAX)
        servo2.set_angle_limits(ANGLE_MIN, ANGLE_MAX)
        print("Angle limits set OK.")
    except ServoTimeoutError as e:
        print(f"ERROR: Servo {e.id_} not responding when setting limits.")
        return

    print("Moving servo 1 to 100 deg...")
    servo1.move(100)
    time.sleep(2)

    print("Moving servo 2 to 100 deg...")
    servo2.move(100)
    time.sleep(2)

    print("Moving both servos back to 60 deg...")
    servo1.move(60)
    servo2.move(60)

    print("Done.")

if __name__ == "__main__":
    main()
