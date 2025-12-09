from pylx16a.lx16a import *
import time

LX16A.initialize("/dev/ttyUSB0")  # 或改成你实际的端口

for i in range(0, 253):
    try:
        s = LX16A(i)
        ang = s.get_physical_angle()
        print(f"Found servo ID {i}, angle={ang}")
        time.sleep(0.05)
    except ServoTimeoutError:
        pass
