from pylx16a.lx16a import *
import time

PORT = "/dev/ttyUSB0"
LX16A.initialize(PORT)

# 只连 7 / 8
hip = LX16A(7)
knee = LX16A(8)

# 限位可以稍微放宽一点，避免卡
hip.set_angle_limits(40, 200)
knee.set_angle_limits(40, 200)

hip_center = 130
knee_center = 60

print("Move LF to center...")
hip.move(hip_center)
knee.move(knee_center)
time.sleep(1.0)

print("Test LF up/down 10 次")
for i in range(10):
    # 抬腿
    hip.move(hip_center - 10)   # 往前一点
    knee.move(knee_center + 15) # 弯一点
    time.sleep(0.6)

    # 放下
    hip.move(hip_center)
    knee.move(knee_center)
    time.sleep(0.6)

print("Done.")
