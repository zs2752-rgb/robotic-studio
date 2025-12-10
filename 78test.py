from pylx16a.lx16a import *
import time

PORT = "/dev/ttyUSB0"
LX16A.initialize(PORT)

# 假装 7/8 是右上腿
hip = LX16A(7)
knee = LX16A(8)

for i in range(3):
    print("pose A")
    hip.move(110)
    knee.move(170)
    time.sleep(1.0)

    print("pose B")
    hip.move(130)
    knee.move(150)
    time.sleep(1.0)
