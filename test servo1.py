from pylx16a.lx16a import *
import time

LX16A.initialize("/dev/ttyUSB0")

try:
    servo1 = LX16A(1)
    servo1.set_angle_limits(0, 240)
    time.sleep(1)
    servo1.move(120)
except ServoTimeoutError as e:
    print(f"Servo {e.id_} is not responding. Exiting...")
