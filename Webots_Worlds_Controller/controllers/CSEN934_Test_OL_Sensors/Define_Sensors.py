LIDAR_PERIOD_MS = 100   # both lidars scan at 10 Hz


def define_sensors(robot, timestep):
    """Enable every sensor of the BMW X5 and return them in a dict."""
    sensors = {}
    for name in ("gps", "gyro", "imu", "accelerometer"):
        dev = robot.getDevice(name)
        if dev is not None:
            dev.enable(timestep)
        sensors[name] = dev

    period = max(timestep, LIDAR_PERIOD_MS)
    for key, name in (("scan", "Sick LMS 291"), ("lidar_3d", "lidar_3d")):
        dev = robot.getDevice(name)
        if dev is not None:
            dev.enable(period)
        sensors[key] = dev
    if sensors["lidar_3d"] is not None:
        sensors["lidar_3d"].enablePointCloud()
    sensors["lidar_period_ms"] = period
    return sensors
