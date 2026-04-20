#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ReactiveAvoidanceNode(Node):
    def __init__(self):
        super().__init__('reactive_avoidance_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/ldlidar_node/scan',
            self.scan_callback,
            10
        )

        # State machine
        self.state = 'FORWARD'

        # Thresholds in meters
        self.stop_threshold = 0.50     # blocked if front is closer than this
        self.clear_threshold = 0.80    # only leave turn/stop once front is above this
        self.too_close_threshold = 0.20  # emergency stop if really close

        self.get_logger().info('Listening to /ldlidar_node/scan')
        self.get_logger().info(f'Initial state: {self.state}')

    def set_state(self, new_state: str):
        if new_state != self.state:
            self.get_logger().info(f'STATE CHANGE: {self.state} -> {new_state}')
            self.state = new_state

    def scan_callback(self, msg: LaserScan):
        front = []
        left = []
        right = []

        for i, distance in enumerate(msg.ranges):
            if math.isnan(distance) or math.isinf(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle) % 360.0

            # front: 340..360 and 0..20
            if angle_deg >= 340.0 or angle_deg <= 20.0:
                front.append(distance)

            # left: 20..100
            elif 20.0 < angle_deg <= 100.0:
                left.append(distance)

            # right: 260..340
            elif 260.0 <= angle_deg < 340.0:
                right.append(distance)

        front_min = min(front) if front else float('inf')
        left_min = min(left) if left else float('inf')
        right_min = min(right) if right else float('inf')

        # -----------------------------
        # State machine with hysteresis
        # -----------------------------

        if self.state == 'FORWARD':
            if front_min < self.too_close_threshold:
                self.set_state('STOP')
            elif front_min < self.stop_threshold:
                if left_min > right_min:
                    self.set_state('TURN_LEFT')
                elif right_min > left_min:
                    self.set_state('TURN_RIGHT')
                else:
                    self.set_state('STOP')

        elif self.state == 'TURN_LEFT':
            # Stay turning until front is clearly open again
            if front_min < self.too_close_threshold:
                self.set_state('STOP')
            elif front_min > self.clear_threshold:
                self.set_state('FORWARD')

        elif self.state == 'TURN_RIGHT':
            # Stay turning until front is clearly open again
            if front_min < self.too_close_threshold:
                self.set_state('STOP')
            elif front_min > self.clear_threshold:
                self.set_state('FORWARD')

        elif self.state == 'STOP':
            # If dangerously close, keep stopping
            if front_min < self.too_close_threshold:
                self.set_state('STOP')
            # If front clears enough, decide direction or move forward
            elif front_min > self.clear_threshold:
                self.set_state('FORWARD')
            elif left_min > right_min and left_min > self.stop_threshold:
                self.set_state('TURN_LEFT')
            elif right_min > left_min and right_min > self.stop_threshold:
                self.set_state('TURN_RIGHT')

        self.get_logger().info(
            f'front={front_min:.3f} m | left={left_min:.3f} m | right={right_min:.3f} m | state={self.state}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveAvoidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
