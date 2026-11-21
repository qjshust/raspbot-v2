"""fake_scan — synthetic LaserScan node (FR-H-04 / SW-I04).

Publishes /scan (frame laser, SensorData QoS) at `rate` Hz. The robot sits
at (0,0) inside a room_x * room_y metre rectangle; 720 rays at 0.5 deg are
intersected with the rectangle (slab method, see raspbot_mock.geometry).
Rays that do not hit the rectangle report range_max. Gaussian noise
(noise parameter) is added to every valid hit.
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from raspbot_mock.geometry import ray_rect_distance


class FakeScan(Node):

    def __init__(self):
        super().__init__('fake_scan')

        self.declare_parameter('rate', 10.0)
        self.declare_parameter('room_x', 5.0)
        self.declare_parameter('room_y', 4.0)
        self.declare_parameter('noise', 0.002)
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('range_min', 0.2)
        self.declare_parameter('range_max', 15.0)
        self.declare_parameter('num_points', 720)

        def p(name):
            return self.get_parameter(name).value

        self.rate = float(p('rate'))
        self.half_x = float(p('room_x')) / 2.0
        self.half_y = float(p('room_y')) / 2.0
        self.noise = float(p('noise'))
        self.frame_id = str(p('frame_id'))
        self.range_min = float(p('range_min'))
        self.range_max = float(p('range_max'))
        self.num_points = int(p('num_points'))

        # 360° / 0.5° = 720 点
        self.angle_increment = 2.0 * math.pi / float(self.num_points)
        self.angles = [
            -math.pi + i * self.angle_increment for i in range(self.num_points)
        ]

        # 契约：/scan 10 Hz, SensorData QoS, frame_id=laser
        self.pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(
            'fake_scan up: rate=%.1f Hz room=%.1fx%.1f m %d points noise=%.4f'
            % (self.rate, self.half_x * 2.0, self.half_y * 2.0,
               self.num_points, self.noise))

    def _tick(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = self.angles[0]
        msg.angle_max = self.angles[-1]
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self.rate
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        ranges = []
        for angle in self.angles:
            dist = ray_rect_distance(angle, self.half_x, self.half_y)
            if dist is None:
                ranges.append(self.range_max)  # 未命中：按 max_range 报告
                continue
            value = dist + random.gauss(0.0, self.noise)
            ranges.append(min(self.range_max, max(self.range_min, value)))
        msg.ranges = ranges
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
