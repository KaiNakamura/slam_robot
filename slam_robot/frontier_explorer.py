import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Empty
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf2_ros import TransformListener, Buffer
from slam_robot.msg import Frontier, FrontierList
import math


class FrontierExplorerNode(Node):
    """Node that autonomously explores by navigating to frontiers."""

    def __init__(self):
        super().__init__("frontier_explorer")

        # Publisher to trigger frontier detection
        self.trigger_publisher = self.create_publisher(Empty, "/get_frontiers", 10)

        # Subscriber for frontiers (MarkerArray format)
        self.frontiers_subscriber = self.create_subscription(
            MarkerArray, "/frontiers", self.frontiers_callback, 10
        )

        # Store latest frontiers (as FrontierList for compatibility)
        self.latest_frontiers = FrontierList()

        # Action client
        self.nav_action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        # Publisher
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/frontier_markers", 10
        )

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State
        self.is_navigating = False
        self.no_frontiers_count = 0
        self.current_goal_handle = None

        # Create timer for exploration loop (1 Hz)
        self.create_timer(1.0, self.explore_callback)

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

        Args:
            p1: First point.
            p2: Second point.

        Returns:
            Distance.
        """
        return math.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)

    def select_best_frontier(self, frontier_list: FrontierList) -> Frontier:
        """Select the best frontier based on distance and size.

        Args:
            frontier_list: List of detected frontiers.

        Returns:
            Best frontier to explore, or None if no frontiers.
        """
        if not frontier_list.frontiers:
            return None

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            # If no robot pose, fallback to first frontier
            return frontier_list.frontiers[0]

        best_frontier = None
        best_cost = float("inf")

        for frontier in frontier_list.frontiers:
            distance = self.euclidean_distance(robot_pose, frontier.centroid)
            if frontier.size > 0:
                cost = distance / frontier.size
            else:
                cost = distance

            if cost < best_cost:
                best_cost = cost
                best_frontier = frontier

        return best_frontier

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

    def publish_frontier_markers(self, frontier_list: FrontierList):
        """Publish visualization markers for frontiers.

        Note: Frontiers are already published as markers by frontier_server,
        but we can republish for visualization consistency.

        Args:
            frontier_list: List of frontiers to visualize.
        """
        marker_array = MarkerArray()

        for i, frontier in enumerate(frontier_list.frontiers):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "frontiers"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = frontier.centroid
            marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        # Delete old markers if no frontiers
        if not frontier_list.frontiers:
            delete_marker = Marker()
            delete_marker.header.frame_id = "map"
            delete_marker.header.stamp = self.get_clock().now().to_msg()
            delete_marker.ns = "frontiers"
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)

        self.marker_publisher.publish(marker_array)

    def frontiers_callback(self, msg: MarkerArray):
        """Handle incoming frontier list (MarkerArray format)."""
        # Convert MarkerArray to FrontierList
        frontier_list = FrontierList()
        for marker in msg.markers:
            if marker.ns == "frontiers" and marker.action == Marker.ADD:
                frontier = Frontier()
                frontier.size = int(marker.scale.x)  # Size stored in scale.x
                frontier.centroid = marker.pose.position
                frontier_list.frontiers.append(frontier)
        self.latest_frontiers = frontier_list

    def explore_callback(self):
        """Main exploration loop callback."""
        if not self.is_navigating:
            # Trigger frontier detection
            self.trigger_publisher.publish(Empty())

            # Use the latest frontiers we received
            frontier_list = self.latest_frontiers

            # Publish markers
            self.publish_frontier_markers(frontier_list)

            if not frontier_list.frontiers:
                self.no_frontiers_count += 1
                self.get_logger().info(
                    f"No frontiers found ({self.no_frontiers_count}/30)"
                )

                # Check termination condition
                if self.no_frontiers_count >= 30:
                    self.get_logger().info(
                        "Exploration complete: No frontiers found for 30 seconds"
                    )
                    # Could shutdown here if desired
                    return
            else:
                # Reset counter
                self.no_frontiers_count = 0

                # Select best frontier
                best_frontier = self.select_best_frontier(frontier_list)
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
