from imu_read import iter_imu_samples,RK4DeadReckoner

dr = RK4DeadReckoner(gravity_world=(0.0, 0.0, 9.80665)) 
it = iter_imu_samples(source= "serial",   # "serial" or "i2c"
    port = "/dev/ttyUSB0",
    baud = 115200,
    timeout = 1.0,
    # i2c params
    i2c_bus = 1,
    i2c_addr = 0x68,
    # output control
    keys= ("acc_g", "gyro_dps"),
    include_all = False,
    add_host_time= False,
    # output rate
    rate_hz= None,
    # integrator
    integrator = None)
for i, d in zip(range(10), it):
    print(i, d)
