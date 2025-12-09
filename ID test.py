from pylx16a.lx16a import *
import time

LX16A.initialize("/dev/ttyUSB0")

for i in range(1, 17):  # 假设最多 16 个舵机
    try:
        s = LX16A(i)
        angle = s.get_physical_angle()  # 随便读一个寄存器测试通信
        print(f"Found servo ID {i}, angle = {angle}")
        time.sleep(0.1)
    except ServoTimeoutError:
        print(f"No response from ID {i}")
