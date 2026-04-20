#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from pymavlink import mavutil


class OdomToMavlink(Node):
    def __init__(self):
        super().__init__('odom_to_mavlink')

        self.declare_parameter('odom_topic', '/odom_rf2o')
        self.declare_parameter('mavlink_connection', 'udp:127.0.0.1:14551')

        self.odom_topic = self.get_parameter('odom_topic').value
        self.mavlink_connection = self.get_parameter('mavlink_connection').value

        self.get_logger().info(f'Connecting to MAVLink at {self.mavlink_connection}')
        self.master = mavutil.mavlink_connection(
            self.mavlink_connection,
            source_system=251,
        )

        self.get_logger().info('Waiting for heartbeat...')
        self.master.wait_heartbeat(timeout=15)

        self.target_system = self.master.target_system
        self.target_component = self.master.target_component

        self.get_logger().info(
            f'Connected to sys={self.target_system} comp={self.target_component}'
        )

        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            50,
        )

        self.last_time = self.get_clock().now()

        self.get_logger().info(f'Subscribed to {self.odom_topic}')

    def odom_callback(self, msg: Odometry):
        # ROS -> MAVLink frame conversion
        px = msg.pose.pose.position.x
        py = -msg.pose.pose.position.y
        pz = -msg.pose.pose.position.z

        vx = msg.twist.twist.linear.x
        vy = -msg.twist.twist.linear.y
        vz = -msg.twist.twist.linear.z

        rollspeed = msg.twist.twist.angular.x
        pitchspeed = -msg.twist.twist.angular.y
        yawspeed = -msg.twist.twist.angular.z

        q_ros = msg.pose.pose.orientation
        q = [
            q_ros.w,
            q_ros.x,
            -q_ros.y,
            -q_ros.z,
        ]

        time_usec = (
            msg.header.stamp.sec * 1_000_000
            + msg.header.stamp.nanosec // 1000
        )

        pose_covariance = [0.01] * 21
        velocity_covariance = [0.01] * 21

        self.master.mav.odometry_send(
            time_usec,
            mavutil.mavlink.MAV_FRAME_LOCAL_FRD,
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            px,
            py,
            pz,
            q,
            vx,
            vy,
            vz,
            rollspeed,
            pitchspeed,
            yawspeed,
            pose_covariance,
            velocity_covariance,
            0,
            mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION,  # important
            100,
        )

        now = self.get_clock().now()
        if (now - self.last_time).nanoseconds > 500_000_000:
            self.get_logger().info(
                f'ODOM sent @ high rate | px={px:.2f} py={py:.2f}'
            )
            self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = OdomToMavlink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
