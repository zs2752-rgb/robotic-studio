from pylx16a.lx16a import *
import time

PORT = "/dev/ttyUSB0"

LX16A.initialize(PORT)
hip = LX16A(7)
knee = LX16A(8)

# 确认开扭矩
hip.enable_torque()
knee.enable_torque()
time.sleep(0.1)

print("Hip torque:", hip.is_torque_enabled())
print("Knee torque:", knee.is_torque_enabled())

# 来回摆动看看有没有力
for i in range(3):
    hip.move(110)
    knee.move(170)
    time.sleep(0.8)

    hip.move(130)
    knee.move(150)
    time.sleep(0.8)
