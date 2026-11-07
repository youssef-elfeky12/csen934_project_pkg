def define_sensors(robot, timestep):
    """Get and enable GPS, gyro and IMU. Returns (gps, gyro, imu)."""
    gps = robot.getDevice("gps")
    gyro = robot.getDevice("gyro")
    imu = robot.getDevice("imu")
    for dev in (gps, gyro, imu):
        dev.enable(timestep)
    return gps, gyro, imu
