from imu_read import iter_imu_samples, RK4DeadReckoner

# PORT = "/dev/ttyUSB0"   # Windows: "COM5"
PORT = "/dev/ttyACM1"
RATE_HZ = 50

dr = RK4DeadReckoner(gravity_world=(0.0, 0.0, 9.80665))

keys = ("t_ms", "roll_deg", "pitch_deg", "rpy_deg", "stationary", "dt_s")

try:
    for s in iter_imu_samples(
        source="serial",
        port=PORT,
        rate_hz=RATE_HZ,
        keys=keys,
        include_all=True,
        integrator=dr,
    ):
        t = s.get("t_ms")
        roll_a = s.get("roll_deg")     # accel-only (firmware)
        pitch_a = s.get("pitch_deg")

        rpy = s.get("rpy_deg")         # integrated (RK4)
        if rpy is None:
            r_i = p_i = y_i = None
        else:
            r_i, p_i, y_i = rpy

        stat = s.get("stationary")
        dt = s.get("dt_s")

        print(
            f"t_ms={t:9.0f}  "
            f"accRP=({roll_a:+7.2f},{pitch_a:+7.2f})  "
            f"intRPY=({r_i:+7.2f},{p_i:+7.2f},{y_i:+7.2f})  "
            f"dt={dt if dt is not None else float('nan'):.3f}  "
            f"still={stat}"
        )

except KeyboardInterrupt:
    print("\nStopped.")
