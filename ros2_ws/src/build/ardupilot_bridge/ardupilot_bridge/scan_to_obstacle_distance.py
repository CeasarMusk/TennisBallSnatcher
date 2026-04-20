#!/usr/bin/env python3

import os
os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'common'

import math
import time
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from pymavlink import mavutil


class ScanToObstacleDistance(Node):
    def __init__(self):
        super().__init__('scan_to_obstacle_distance')

        self.subscription = self.create_subscription(
            LaserScan,
            '/ldlidar_node/scan',
            self.scan_callback,
            10
        )

        # MAVProxy forwards to this local UDP port.
        # We should LISTEN here, not send to it.
        self.mav_endpoint = 'udpin:127.0.0.1:14551'

        self.get_logger().info(f'Listening for MAVLink from MAVProxy at {self.mav_endpoint}')
        self.master = mavutil.mavlink_connection(self.mav_endpoint)

        self.get_logger().info('Waiting for heartbeat...')
        hb = self.master.wait_heartbeat(timeout=15)
        if hb is None:
            raise RuntimeError('No heartbeat received from MAVProxy/Pixhawk on UDP 14551')

        self.get_logger().info(
            f'Connected to system={self.master.target_system}, component={self.master.target_component}'
        )

        self.unknown_distance_cm = 65535
        self.num_sectors = 8
        self.sector_width_deg = 360.0 / self.num_sectors

        self.last_send_time = 0.0
        self.send_interval_sec = 0.10  # 10 Hz

    def normalize_angle_deg(self, angle_rad: float) -> float:
        return math.degrees(angle_rad) % 360.0

    def valid_range(self, distance: float, msg: LaserScan) -> bool:
        return (
            not math.isnan(distance)
            and not math.isinf(distance)
            and msg.range_min <= distance <= msg.range_max
        )

    def scan_callback(self, msg: LaserScan) -> None:
        now = time.time()
        if now - self.last_send_time < self.send_interval_sec:
            return
        self.last_send_time = now

        sector_bins: List[List[float]] = [[] for _ in range(self.num_sectors)]

        for i, distance in enumerate(msg.ranges):
            if not self.valid_range(distance, msg):
                continue

            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = self.normalize_angle_deg(angle_rad)

            sector_index = int(angle_deg // self.sector_width_deg) % self.num_sectors
            sector_bins[sector_index].append(distance)

        distances_cm: List[int] = []

        for values in sector_bins:
            if not values:
                distances_cm.append(self.unknown_distance_cm)
                continue

            min_distance_m = min(values)
            min_distance_cm = int(min_distance_m * 100.0)
            min_distance_cm = max(0, min(min_distance_cm, 65534))
            distances_cm.append(min_distance_cm)

        distances_array = distances_cm + [self.unknown_distance_cm] * (72 - len(distances_cm))

        try:
            self.master.mav.obstacle_distance_send(
                int(time.time() * 1_000_000),          # time_usec
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                distances_array,
                0,                                     # increment (deprecated int field)
                int(msg.range_min * 100),              # min distance cm
                int(msg.range_max * 100),              # max distance cm
                45.0,                                  # increment_f in degrees
                0.0,                                   # angle_offset
                mavutil.mavlink.MAV_FRAME_BODY_FRD
            )

            self.get_logger().info(f'Sent sectors (cm): {distances_cm}')

        except Exception as e:
            self.get_logger().error(f'Failed to send MAVLink obstacle data: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ScanToObstacleDistance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
