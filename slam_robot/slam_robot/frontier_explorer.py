import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, Quaternion
from tf2_ros import TransformListener, Buffer
from slam_robot_interfaces.msg import Frontier, FrontierList
import math


class FrontierExplorerNode(Node):
    """Node that autonomously explores by navigating to frontiers."""

    def __init__(self):
        super().__init__("frontier_explorer")

        # Subscribe to frontiers topic
        self.frontiers_sub = self.create_subscription(
            FrontierList, "/frontiers", self.frontiers_callback, 10
        )

        # Action client
        self.nav_action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State
        self.is_navigating = False
        self.no_frontiers_count = 0
        self.current_goal_handle = None

        self.get_logger().info("Frontier explorer node started")

    def get_robot_pose(self) -> Point:
        """Get current robot position via TF.

        Returns:
            Robot position as Point in world coordinates.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            return transform.transform.translation
        except Exception as e:
            self.get_logger().error(f"Failed to get robot pose: {e}")
            return None

    def euclidean_distance(self, p1: Point, p2: Point) -> float:
        """Calculate Euclidean distance between two points.

        TODO: Probably a better way to do this?

        Args:
            p1: First point.
            p2: Second point.

        Returns:
            Distance.
        """
        return math.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)

    def compute_frontier_cost(
        self,
        frontier: Frontier,
        robot_pose: Point,
        distance_weight: float = 1.0,
        size_weight: float = 1.0,
        min_distance: float = 0.5,
    ) -> float:
        """Compute cost for a frontier (lower is better).

        TODO: Refactor all constants into a parameter file.

        Args:
            frontier: The frontier to evaluate.
            robot_pose: Current robot position in world coordinates.
            distance_weight: Weight for distance factor.
            size_weight: Weight for size factor.
            min_distance: Minimum distance threshold in meters.

        Returns:
            Cost value (lower is better).
        """
        distance = self.euclidean_distance(robot_pose, frontier.centroid)

        # Reject frontiers with no size
        if frontier.size <= 0:
            return float("inf")

        # Reject frontiers that are too close
        if distance < min_distance:
            return float("inf")

        return (distance_weight * distance) / (size_weight * frontier.size)

    def select_best_frontier(self, frontier_list: FrontierList) -> Frontier:
        """Select the best frontier using cost function.

        Args:
            frontier_list: List of detected frontiers.

        Returns:
            Best frontier to explore, or None if no frontiers.
        """
        if not frontier_list.frontiers:
            return None

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            # Fallback to first frontier if pose unavailable
            return frontier_list.frontiers[0]

        # Select best frontier (lowest cost)
        return min(
            frontier_list.frontiers,
            key=lambda f: self.compute_frontier_cost(f, robot_pose),
        )

    def send_navigation_goal(self, frontier: Frontier):
        """Send navigation goal to Nav2 action server.

        Args:
            frontier: The frontier to navigate to.
        """
        if not self.nav_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Nav2 action server not available")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position = frontier.centroid
        goal_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.get_logger().info(
            f"Sending navigation goal to frontier at ({frontier.centroid.x:.2f}, {frontier.centroid.y:.2f})"
        )

        send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.navigation_result_callback)

        self.is_navigating = True

    def navigation_result_callback(self, future):
        """Handle navigation goal result.

        Args:
            future: Future containing the goal handle.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Navigation goal was rejected")
            self.is_navigating = False
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info("Navigation goal accepted")

        # Get result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.navigation_get_result_callback)

    def navigation_get_result_callback(self, future):
        """Handle navigation result.

        Args:
            future: Future containing the result.
        """
        result = future.result().result
        status = future.result().status

        self.is_navigating = False

        if status == 4:  # SUCCEEDED
            self.get_logger().info("Navigation goal succeeded")
        elif status == 2:  # CANCELED
            self.get_logger().warn("Navigation goal was canceled")
        elif status == 3:  # ABORTED
            self.get_logger().warn("Navigation goal was aborted")
        else:
            self.get_logger().warn(f"Navigation goal ended with status: {status}")

    def frontiers_callback(self, msg: FrontierList):
        """Handle incoming frontiers from topic.

        Args:
            msg: FrontierList message.
        """
        if self.is_navigating:
            return  # Don't interrupt current navigation

        # TODO: Remove hardcoded values
        if not msg.frontiers:
            self.no_frontiers_count += 1
            self.get_logger().info(f"No frontiers found ({self.no_frontiers_count}/30)")

            # Check termination condition
            if self.no_frontiers_count >= 30:
                self.get_logger().info(
                    "Exploration complete: No frontiers found for 30 updates"
                )
                return
        else:
            # Reset counter
            self.no_frontiers_count = 0

            # Select best frontier
            best_frontier = self.select_best_frontier(msg)
            if best_frontier:
                # Wait for action client to be ready
                if self.nav_action_client.wait_for_server(timeout_sec=0.5):
                    self.send_navigation_goal(best_frontier)
                else:
                    self.get_logger().warn("Nav2 action server not ready")


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
