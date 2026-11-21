"""raspbot_mock — mock nodes replacing the RASPBOT-V2 hardware drivers.

fake_base: /cmd_vel_safe -> /odom_raw (30 Hz, open-loop integration)
fake_imu : /imu/data (100 Hz, synthetic yaw)
fake_scan: /scan (10 Hz, synthetic 5x4 m room)
"""
