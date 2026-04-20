#!/usr/bin/env python3

from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pymavlink import mavutil


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class CmdVelToGuided(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_to_guided')

        self.declare_parameter('mavlink_connection', 'udp:127.0.0.1:14550')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('send_rate_hz', 20.0)
        self.declare_parameter('command_timeout_s', 0.5)
        self.declare_parameter('max_vx', 0.35)
        self.declare_parameter('max_vy', 0.35)
        self.declare_parameter('max_yaw_rate', 0.70)
        self.declare_parameter('target_system', 0)
        self.declare_parameter('target_component', 0)

        self.mavlink_connection = self.get_parameter('mavlink_connection').value
        self.baud = int(self.get_parameter('baud').value)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.send_rate_hz = float(self.get_parameter('send_rate_hz').value)
        self.command_timeout_s = float(self.get_parameter('command_timeout_s').value)
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)

        self.user_target_system = int(self.get_parameter('target_system').value)
        self.user_target_component = int(self.get_parameter('target_component').value)

        self.last_twist: Optional[Twist] = None
        self.last_twist_time = None
        self.last_status_log_ns = 0

        self.get_logger().info(f'Connecting to MAVLink: {self.mavlink_connection}')
        self.master = mavutil.mavlink_connection(
            self.mavlink_connection,
            baud=self.baud,
            source_system=250,
        )

        self.get_logger().info('Waiting for heartbeat...')
        self.master.wait_heartbeat(timeout=15)

        self.target_system = self.user_target_system or self.master.target_system
        self.target_component = self.user_target_component or self.master.target_component

        self.get_logger().info(
            f'Connected. target_system={self.target_system}, '
            f'target_component={self.target_component}'
        )

        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_cb, 10)
        self.timer = self.create_timer(1.0 / self.send_rate_hz, self.send_setpoint)

        self.get_logger().info(
            f'Subscribed to {self.cmd_vel_topic} and forwarding to ArduPilot GUIDED velocity control'
        )

    def cmd_vel_cb(self, msg: Twist) -> None:
        self.last_twist = msg
        self.last_twist_time = self.get_clock().now()

    def send_setpoint(self) -> None:
        now = self.get_clock().now()

        if self.last_twist is None or self.last_twist_time is None:
            vx = 0.0
            vy = 0.0
            yaw_rate = 0.0
            stale = True
        else:
            age_s = (now - self.last_twist_time).nanoseconds / 1e9
            stale = age_s > self.command_timeout_s

            if stale:
                vx = 0.0
                vy = 0.0
                yaw_rate = 0.0
            else:
                # ROS REP-103 body convention is x forward, y left, z up.
                # MAVLink BODY_NED is x forward, y right, z down.
                # So Y and yaw-rate are flipped here.
                vx = clamp(self.last_twist.linear.x, -self.max_vx, self.max_vx)
                vy = clamp(-self.last_twist.linear.y, -self.max_vy, self.max_vy)
                yaw_rate = clamp(-self.last_twist.angular.z, -self.max_yaw_rate, self.max_yaw_rate)

        # Velocity + yaw_rate only:
        # ignore position, ignore accel, ignore yaw, use vx/vy/vz and yaw_rate
        type_mask = 1479

        time_boot_ms = int(now.nanoseconds / 1_000_000) & 0xFFFFFFFF

        self.master.mav.set_position_target_local_ned_send(
            time_boot_ms,
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0.0, 0.0, 0.0,   # x, y, z position ignored
            vx, vy, 0.0,      # fixed-height for now, so vz = 0
            0.0, 0.0, 0.0,    # acceleration ignored
            0.0,              # yaw ignored
            yaw_rate,
        )

        # Log roughly once per second
        if now.nanoseconds - self.last_status_log_ns > 1_000_000_000:
            state = 'STALE->STOP' if stale else 'ACTIVE'
            self.get_logger().info(
                f'{state} vx={vx:.2f} vy={vy:.2f} yaw_rate={yaw_rate:.2f}'
            )
            self.last_status_log_ns = now.nanoseconds


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelToGuided()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down cmd_vel_to_guided')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
