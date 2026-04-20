#!/usr/bin/env python3

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose_stamped


class MoveBaseSimpleToNav2(Node):
    def __init__(self) -> None:
        super().__init__("move_base_simple_to_nav2")

        self._subscription = self.create_subscription(
            PoseStamped,
            "/move_base_simple/goal",
            self.goal_callback,
            10,
        )

        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.get_logger().info("Bridge ready: /move_base_simple/goal -> /navigate_to_pose")

    def goal_callback(self, msg: PoseStamped) -> None:
        self.get_logger().info(
            f"Received Foxglove goal in frame '{msg.header.frame_id}' "
            f"at x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}"
        )

        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("/navigate_to_pose action server is not available")
            return

        try:
            transformed_goal = self.transform_goal_to_map(msg)
        except TransformException as exc:
            self.get_logger().error(f"Failed to transform goal to map frame: {exc}")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = transformed_goal

        self.get_logger().info(
            f"Sending Nav2 goal in frame '{goal_msg.pose.header.frame_id}' "
            f"at x={goal_msg.pose.pose.position.x:.3f}, "
            f"y={goal_msg.pose.pose.position.y:.3f}"
        )

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def transform_goal_to_map(self, msg: PoseStamped) -> PoseStamped:
        input_goal = PoseStamped()
        input_goal.header = msg.header
        input_goal.pose = msg.pose

        if input_goal.header.frame_id == "":
            input_goal.header.frame_id = "map"

        if input_goal.header.stamp.sec == 0 and input_goal.header.stamp.nanosec == 0:
            input_goal.header.stamp = self.get_clock().now().to_msg()

        if input_goal.header.frame_id == "map":
            return input_goal

        transform = self._tf_buffer.lookup_transform(
            "map",
            input_goal.header.frame_id,
            rclpy.time.Time(),
            timeout=Duration(seconds=1.0),
        )

        transformed_goal = do_transform_pose_stamped(input_goal, transform)
        transformed_goal.header.frame_id = "map"
        transformed_goal.header.stamp = self.get_clock().now().to_msg()
        transformed_goal.pose.position.z = 0.0

        return transformed_goal

    def goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to send goal: {exc}")
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected the goal")
            return

        self.get_logger().info("Nav2 accepted the goal")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Feedback: x={feedback.current_pose.pose.position.x:.3f}, "
            f"y={feedback.current_pose.pose.position.y:.3f}, "
            f"remaining={feedback.distance_remaining:.3f}"
        )

    def result_callback(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to get result: {exc}")
            return

        status = result.status
        nav_result = result.result

        self.get_logger().info(
            f"Goal finished. status={status}, "
            f"error_code={nav_result.error_code}, "
            f"error_msg='{nav_result.error_msg}'"
        )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MoveBaseSimpleToNav2()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
