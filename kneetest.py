for sid in [2,4,6,8]:
    a = servo[sid].get_physical_angle()
    servo[sid].move(a + 20)
time.sleep(1)
for sid in [2,4,6,8]:
    a = servo[sid].get_physical_angle()
    servo[sid].move(a)
