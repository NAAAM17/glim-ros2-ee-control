#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, MotionPlanRequest
from shape_msgs.msg import SolidPrimitive


class MoveEeByInput(Node):
    def __init__(self):
        super().__init__('move_ee_by_input')

        self.client = ActionClient(self, MoveGroup, '/move_action')

        self.planning_group = "manipulator"
        self.base_frame = "base_link"
        self.ee_link = "link_6"

        self.pos_tolerance = 0.02  # 2 cm

        self.get_logger().info("Waiting for /move_action action server...")
        self.client.wait_for_server()
        self.get_logger().info("Ready. Enter: x y z (meters). ex) 0.30 0.00 0.40")

    def build_constraints(self, x: float, y: float, z: float) -> Constraints:
        pos = PositionConstraint()
        pos.header.frame_id = self.base_frame
        pos.link_name = self.ee_link
        pos.weight = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.pos_tolerance]

        pos.constraint_region.primitives.append(sphere)

        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        p.orientation.w = 1.0

        pos.constraint_region.primitive_poses.append(p)

        c = Constraints()
        c.position_constraints.append(pos)
        return c

    def send_goal(self, x: float, y: float, z: float) -> None:
        constraints = self.build_constraints(x, y, z)

        req = MotionPlanRequest()
        req.group_name = self.planning_group
        req.goal_constraints.append(constraints)
        req.allowed_planning_time = 15.0
        req.num_planning_attempts = 20

        goal_msg = MoveGroup.Goal()
        goal_msg.request = req

        self.get_logger().info(
            f"PlanningGroup={self.planning_group}, EE={self.ee_link}, Frame={self.base_frame} | "
            f"Target=({x:.3f}, {y:.3f}, {z:.3f}) m"
        )

        send_future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal rejected.")
            return

        self.get_logger().info("Goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val == 1:
            self.get_logger().info("SUCCESS (planned & executed by move_group).")
        else:
            self.get_logger().error(f"FAILED. MoveItErrorCode={result.error_code.val}")

    def prompt_and_move(self) -> bool:
        try:
            s = input("target(x y z)> ").strip()
            if not s:
                return True
            if s.lower() in ("q", "quit", "exit"):
                return False

            parts = s.split()
            if len(parts) != 3:
                self.get_logger().error("Input must be 3 numbers: x y z")
                return True

            x, y, z = map(float, parts)
            self.send_goal(x, y, z)
            return True

        except ValueError:
            self.get_logger().error("Parse error. Example: 0.30 0.00 0.40")
            return True
        except Exception as e:
            self.get_logger().error(f"Error: {type(e).__name__}: {e}")
            return True


def main():
    rclpy.init()
    node = MoveEeByInput()

    running = True
    while rclpy.ok() and running:
        running = node.prompt_and_move()
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
