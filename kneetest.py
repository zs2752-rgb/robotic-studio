for sid in [2,4,6,8]:
    a = servos[sid].get_physical_angle()
    servos[sid].move(a + 20)
time.sleep(1)
for sid in [2,4,6,8]:
    a = servos[sid].get_physical_angle()
    servos[sid].move(a)
