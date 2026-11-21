"""fake_imu — synthetic IMU node (FR-H-04 / SW-I04).

Publishes /imu/data (frame imu_link, SensorData QoS) at `rate` Hz.
Yaw follows yaw = yaw_rate_deg_s * t (default 0.0: stationary, pointing 0).
The quaternion is generated from yaw only: z = sin(yaw/2), w = cos(yaw/2).
Covariance defaults match raspbot_imu: yaw 0.0025, tilt 1.0, gyro 0.0004,
accel 0.04 (design doc §7.2).
"""

import math
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class FakeImu(Node):

    def __init__(self):
        super().__init__('fake_imu')

        self.declare_parameter('rate', 100.0)
        self.declare_parameter('noise', 0.0)
        self.declare_parameter('yaw_rate_deg_s', 0.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('yaw_variance', 0.0025)
        self.declare_parameter('tilt_variance', 1.0)
        self.declare_parameter('gyro_variance', 0.0004)
        self.declare_parameter('accel_variance', 0.04)

        def p(name):
            return self.get_parameter(name).value

        self.rate = float(p('rate'))
        self.noise = float(p('noise'))
        self.wz = math.radians(float(p('yaw_rate_deg_s')))
        self.frame_id = str(p('frame_id'))
        self.yaw_var = float(p('yaw_variance'))
        self.tilt_var = float(p('tilt_variance'))
        self.gyro_var = float(p('gyro_variance'))
        self.accel_var = float(p('accel_variance'))

        self._start = time.monotonic()

        # 契约：/imu/data 100 Hz, SensorData QoS
        self.pub = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(
            'fake_imu up: rate=%.1f Hz yaw_rate=%.2f deg/s noise=%.4f'
            % (self.rate, math.degrees(self.wz), self.noise))

    def _tick(self):
        t = time.monotonic() - self._start
        yaw = self.wz * t + random.gauss(0.0, self.noise)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation.z = math.sin(yaw / 2.0)
        msg.orientation.w = math.cos(yaw / 2.0)
        # 协方差对角线：x/y = roll/pitch（tilt），z = yaw
        msg.orientation_covariance[0] = self.tilt_var
        msg.orientation_covariance[7] = self.tilt_var
        msg.orientation_covariance[35] = self.yaw_var
        msg.angular_velocity.z = self.wz
        msg.angular_velocity_covariance[0] = self.gyro_var
        msg.angular_velocity_covariance[7] = self.gyro_var
        msg.angular_velocity_covariance[35] = self.gyro_var
        msg.linear_acceleration_covariance[0] = self.accel_var
        msg.linear_acceleration_covariance[7] = self.accel_var
        msg.linear_acceleration_covariance[35] = self.accel_var
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
