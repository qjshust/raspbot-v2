"""ROS 2 节点：读取 IMU 数据源，解算姿态，发布 sensor_msgs/Imu。

发布：``imu/data``（默认 100 Hz），frame_id 默认 ``imu_link``。
参数见 ``config/imu.yaml``；换传感器只改 ``source`` 与对应连接参数。
"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu

from .attitude import (
    MahonyAHRS,
    YawEstimator,
    estimate_gyro_bias,
    quat_from_yaw,
    quat_multiply,
)
from .config import validate_config
from .msgs import build_imu_dict
from .sources import ImuSource, make_source


class ImuNode(Node):
    """IMU 驱动节点。"""

    def __init__(self) -> None:
        super().__init__('imu_node')

        self.declare_parameter('source', 'icm42688')
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('mode', 'yaw')  # yaw | mahony
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('calibrate_seconds', 5.0)
        self.declare_parameter('yaw_offset_deg', 0.0)
        # 航向来源：device=用模块自带的解算角度（HWT101 推荐）；gyro=自己积分
        self.declare_parameter('angle_source', 'gyro')

        # 串口（WitMotion / HWT101）
        self.declare_parameter('serial_port', '/dev/raspbot_imu')
        self.declare_parameter('baud', 115200)

        # SPI（ICM-42688-P）
        self.declare_parameter('spi_bus', 0)
        self.declare_parameter('spi_device', 0)
        self.declare_parameter('spi_speed_hz', 2_000_000)
        self.declare_parameter('gyro_fs_dps', 500.0)
        self.declare_parameter('accel_fs_g', 4.0)
        self.declare_parameter('odr_hz', 1000.0)

        # Mahony 增益
        self.declare_parameter('mahony_kp', 0.8)
        self.declare_parameter('mahony_ki', 0.02)

        # 协方差（供 robot_localization 使用）
        self.declare_parameter('orientation_yaw_variance', 0.0025)
        self.declare_parameter('orientation_tilt_variance', 1.0)
        self.declare_parameter('angular_velocity_variance', 0.0004)
        self.declare_parameter('linear_acceleration_variance', 0.04)

        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.yaw_offset = math.radians(float(self.get_parameter('yaw_offset_deg').value))

        # 读取异常统计与限频告警状态；必须先于 _calibrate 初始化
        self._errors = 0
        self._last_error_report = time.monotonic()

        # 参数校验（SW-U15 / SW-G04）：非法值抛 ValueError 且信息含可选值
        self.mode, self.angle_source, self._source_kind = validate_config(
            str(self.get_parameter('mode').value),
            str(self.get_parameter('angle_source').value),
            str(self.get_parameter('source').value),
        )

        if self.angle_source == 'device' and self.mode == 'mahony':
            self.get_logger().warning('mode=mahony 会忽略器件角度（只用角速度+加速度），'
                                   '要使用 HWT101 的解算角度请设 mode=yaw')

        self.source: ImuSource = make_source(self._source_kind, **self._source_kwargs())

        bias = self._calibrate(float(self.get_parameter('calibrate_seconds').value))

        self._mahony: MahonyAHRS | None = None
        self._yaw_est: YawEstimator | None = None
        if self.mode == 'mahony':
            self._mahony = MahonyAHRS(
                kp=float(self.get_parameter('mahony_kp').value),
                ki=float(self.get_parameter('mahony_ki').value),
            )
            if bias is not None:
                self._mahony.bias = bias
        elif self.angle_source == 'gyro':
            self._yaw_est = YawEstimator(bias=bias if bias is not None else None)
        else:
            # 器件角度：不需要我们自己积分，也不需要陀螺零偏
            self._yaw_est = None
            self.get_logger().info('航向直接取自 HWT101 的解算角度（angle_source=device）')

        self.publisher = self.create_publisher(Imu, 'imu/data', 10)
        self._last_t: float | None = None
        self._misses = 0
        self._frames = 0
        self._last_report = time.monotonic()

        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(
            f"IMU 节点启动：source={self._source_kind} "
            f"mode={self.mode} rate={self.rate_hz:.0f} Hz frame={self.frame_id}"
        )

    # --- 初始化辅助 ---
    def _source_kwargs(self) -> dict:
        kind = self._source_kind
        if kind == 'mock':
            return {'rate_hz': self.rate_hz}
        if kind == 'witmotion':
            return {
                'port': str(self.get_parameter('serial_port').value),
                'baud': int(self.get_parameter('baud').value),
            }
        if kind == 'icm42688':
            return {
                'bus': int(self.get_parameter('spi_bus').value),
                'device': int(self.get_parameter('spi_device').value),
                'speed_hz': int(self.get_parameter('spi_speed_hz').value),
                'gyro_fs_dps': float(self.get_parameter('gyro_fs_dps').value),
                'accel_fs_g': float(self.get_parameter('accel_fs_g').value),
                'odr_hz': float(self.get_parameter('odr_hz').value),
            }
        raise ValueError(f"未知 source：{kind}")

    def _calibrate(self, seconds: float):
        """静止采样估计陀螺零偏；失败返回 None 并继续用 0 零偏。"""
        if seconds <= 0.0:
            self.get_logger().warning('已跳过零偏标定（calibrate_seconds<=0），航向可能漂移')
            return None
        self.get_logger().info(f'零偏标定：保持车辆静止 {seconds:.1f} s ...')
        samples = []
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            try:
                sample = self.source.read()
            except Exception as exc:  # noqa: BLE001 - 标定期间源异常不退出
                self._handle_source_error('标定读取', exc)
                time.sleep(0.005)
                continue
            if sample is None:
                time.sleep(0.005)
                continue
            samples.append(sample.gyro)
        try:
            bias = estimate_gyro_bias(samples)
        except ValueError as exc:
            self.get_logger().warning(f'{exc}，本次使用 0 零偏')
            return None
        self.get_logger().info(
            '零偏估计 (rad/s)：'
            f'x={bias[0]:+.5f} y={bias[1]:+.5f} z={bias[2]:+.5f} '
            f'（{len(samples)} 帧）'
        )
        return bias

    # --- 主循环 ---
    def _handle_source_error(self, where: str, exc: Exception) -> None:
        """数据源异常统一处理：计数 + 限频 WARN（≤1 次/5 s），不发假数据。"""
        self._errors += 1
        now = time.monotonic()
        if now - self._last_error_report >= 5.0:
            self._last_error_report = now
            self.get_logger().warning(
                f'IMU 数据源异常（{where}）：{exc!r}；累计 {self._errors} 次。'
                '停止发布 /imu/data，源恢复后自动继续'
            )

    def _tick(self) -> None:
        try:
            sample = self.source.read()
        except Exception as exc:  # noqa: BLE001 - D-05：异常不退出、不发假值
            self._last_t = None
            self._handle_source_error('读取', exc)
            return

        if sample is None:
            self._misses += 1
            self._report()
            return

        try:
            dt = 1.0 / self.rate_hz if self._last_t is None else sample.t - self._last_t
            if dt <= 1e-6 or dt > 0.2:
                self.get_logger().warning(f'采样间隔异常 dt={dt:.4f} s，按标称周期处理')
                dt = 1.0 / self.rate_hz

            if self.mode == 'mahony' and self._mahony is not None:
                quat = self._mahony.update(sample.gyro, sample.accel, dt)
            elif self.angle_source == 'device' and sample.yaw_deg is not None:
                # 器件已经做过卡尔曼融合，直接用它的航向，不做二次积分
                quat = quat_from_yaw(math.radians(sample.yaw_deg))
            else:
                if self._yaw_est is None:  # device 模式下偶尔丢角度帧 → 退回积分
                    self._yaw_est = YawEstimator()
                quat = quat_from_yaw(self._yaw_est.update(sample.gyro, sample.accel, dt))

            if self.yaw_offset:
                quat = quat_multiply(quat_from_yaw(self.yaw_offset), quat)

            tilt_var = float(self.get_parameter('orientation_tilt_variance').value)
            yaw_var = float(self.get_parameter('orientation_yaw_variance').value)
            gyro_var = float(self.get_parameter('angular_velocity_variance').value)
            accel_var = float(self.get_parameter('linear_acceleration_variance').value)
            now = self.get_clock().now()
            data = build_imu_dict(
                float(now.nanoseconds) * 1e-9,
                self.frame_id,
                quat,
                sample.gyro,
                sample.accel,
                yaw_var,
                tilt_var,
                gyro_var,
                accel_var,
            )

            msg = Imu()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = data['frame_id']
            msg.orientation.w = data['orientation'][0]
            msg.orientation.x = data['orientation'][1]
            msg.orientation.y = data['orientation'][2]
            msg.orientation.z = data['orientation'][3]
            msg.angular_velocity.x = data['angular_velocity'][0]
            msg.angular_velocity.y = data['angular_velocity'][1]
            msg.angular_velocity.z = data['angular_velocity'][2]
            msg.linear_acceleration.x = data['linear_acceleration'][0]
            msg.linear_acceleration.y = data['linear_acceleration'][1]
            msg.linear_acceleration.z = data['linear_acceleration'][2]
            msg.orientation_covariance = data['orientation_covariance']
            msg.angular_velocity_covariance = data['angular_velocity_covariance']
            msg.linear_acceleration_covariance = data['linear_acceleration_covariance']
            self.publisher.publish(msg)
        except Exception as exc:  # noqa: BLE001 - D-05：处理/发布异常不退出
            self._last_t = None
            self._handle_source_error('处理/发布', exc)
            return

        self._last_t = sample.t
        self._frames += 1
        self._report()

    def _report(self) -> None:
        now = time.monotonic()
        if now - self._last_report < 5.0:
            return
        elapsed = now - self._last_report
        measured = self._frames / elapsed if elapsed > 0 else 0.0
        self.get_logger().info(
            f'发布 {measured:.1f} Hz（目标 {self.rate_hz:.0f} Hz），空读 {self._misses} 次，'
            f'读取异常累计 {self._errors} 次'
        )
        self._frames = 0
        self._misses = 0
        self._last_report = now

    def destroy_node(self) -> bool:
        source = getattr(self, 'source', None)
        if source is not None:
            try:
                source.close()
            except Exception:  # noqa: BLE001 - 关闭失败不应影响退出
                pass
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # SIGTERM/SIGINT 触发的优雅关闭（D-06 / CODE-04）：干净收尾、退出码 0
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
