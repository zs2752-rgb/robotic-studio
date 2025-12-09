from pylx16a.lx16a import *
import time

LX16A.initialize("/dev/ttyUSB0")

for i in range(0, 253):  # LX-16A ID 范围一般是 0~252
    try:
        s = LX16A(i)
        angle = s.get_physical_angle()   # 随便读一个寄存器测试
        print(f"Found servo ID {i}, angle = {angle}")
        time.sleep(0.05)
    except ServoTimeoutError:
        # 这个 ID 上没人
        pass
