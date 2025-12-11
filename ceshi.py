from pylx16a.lx16a import *
import time, math

PORT = "/dev/ttyUSB0"
LX16A.initialize(PORT)
hips = {i: LX16A(i) for i in (1,3,5,7)}  # 四个“髋”

for s in hips.values():
    s.set_angle_limits(40, 200)

hip0 = {1:110, 3:110, 5:110, 7:110}   # 暂时都设成一样
AMP  = 8
STEP_TIME = 0.03

for k in range(300):
    phi = 2*math.pi * (k/60.0)
    swing = math.sin(phi)
    for sid in (1,3,5,7):
        a = hip0[sid] + AMP * swing
        hips[sid].move(a)
    time.sleep(STEP_TIME)
