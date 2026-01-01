import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import ComputePathToPose, NavigateToPose
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

        # Callback group for action clients (allows reentrant calls)
        self.action_cb_group = ReentrantCallbackGroup()

        # Action clients
        self.nav_action_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            callback_group=self.action_cb_group,
        )
        self.compute_path_client = ActionClient(
            self,
            ComputePathToPose,
            "/compute_path_to_pose",
            callback_group=self.action_cb_group,
        )

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State
        self.is_navigating = False
        self.no_frontiers_count = 0
        self.current_goal_handle = None

        # Async path computation state
        self.pending_path_count = 0
        self.path_distances = {}  # frontier_id -> distance
        self.path_timers = {}  # frontier_id -> Timer
        self.current_frontiers = None
        self.robot_pose = None

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
            # Convert Vector3 to Point (TF returns Vector3, but we need Point for Pose)
            translation = transform.transform.translation
            return Point(x=translation.x, y=translation.y, z=translation.z)
        except Exception as e:
            self.get_logger().error(f"Failed to get robot pose: {e}")
            return None

    def compute_path_async(self, frontier: Frontier, start: Point):
        """Start async path computation for a single frontier.

        Args:
            frontier: The frontier to compute path to.
            start: Robot's current position.
        """
        # Check if server is available
        if not self.compute_path_client.wait_for_server(timeout_sec=0.1):
            # Server not available, mark as unreachable
            frontier_id = id(frontier)
            self.path_distances[frontier_id] = float("inf")
            self.pending_path_count -= 1
            if self.pending_path_count == 0:
                self.select_and_navigate()
            return

        # Build goal message
        goal_msg = ComputePathToPose.Goal()
        goal_msg.start.header.frame_id = "map"
        goal_msg.start.header.stamp = self.get_clock().now().to_msg()
        goal_msg.start.pose.position = start
        goal_msg.goal.header.frame_id = "map"
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.pose.position = frontier.centroid

        # Send goal async
        future = self.compute_path_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f, fr=frontier: self.path_goal_callback(f, fr))

        # Create timeout timer
        timer = self.create_timer(
            5.0, lambda fid=id(frontier): self.path_timeout_callback(fid)
        )
        self.path_timers[id(frontier)] = timer

    def path_goal_callback(self, future, frontier: Frontier):
        """Handle goal acceptance/rejection for path computation.

        Args:
            future: Future from send_goal_async.
            frontier: Frontier object (captured via lambda).
        """
        goal_handle = future.result()
        frontier_id = id(frontier)

        if not goal_handle or not goal_handle.accepted:
            # Goal rejected, mark as unreachable
            if frontier_id in self.path_timers:
                self.path_timers[frontier_id].cancel()
                del self.path_timers[frontier_id]
            self.path_distances[frontier_id] = float("inf")
            self.pending_path_count -= 1
            if self.pending_path_count == 0:
                self.select_and_navigate()
            return

        # Goal accepted, get result async
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, fr=frontier: self.path_result_callback(f, fr)
        )

    def path_result_callback(self, future, frontier: Frontier):
        """Handle path computation result.

        Args:
            future: Future from get_result_async.
            frontier: Frontier object (captured via lambda).
        """
        frontier_id = id(frontier)

        # Check if already timed out (timer may have fired)
        if frontier_id in self.path_distances and self.path_distances[
            frontier_id
        ] == float("inf"):
            return  # Already handled by timeout

        # Cancel timer
        if frontier_id in self.path_timers:
            self.path_timers[frontier_id].cancel()
            del self.path_timers[frontier_id]

        # Get result
        result = future.result().result
        path = result.path.poses

        # Check path validity
        if len(path) < 2:
            self.path_distances[frontier_id] = float("inf")
            self.pending_path_count -= 1
            if self.pending_path_count == 0:
                self.select_and_navigate()
            return

        # Compute path distance (sum distances between consecutive poses)
        total_distance = 0.0
        for i in range(len(path) - 1):
            p1 = path[i].pose.position
            p2 = path[i + 1].pose.position
            total_distance += math.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)

        # Store distance
        self.path_distances[frontier_id] = total_distance
        self.pending_path_count -= 1
        if self.pending_path_count == 0:
            self.select_and_navigate()

    def path_timeout_callback(self, frontier_id: int):
        """Handle timeout for individual path computation.

        Args:
            frontier_id: ID of the frontier (passed via lambda).
        """
        # Check if already completed (result callback may have fired first)
        if frontier_id in self.path_distances and self.path_distances[
            frontier_id
        ] != float("inf"):
            return  # Already completed

        # Set distance to inf
        self.path_distances[frontier_id] = float("inf")

        # Remove timer from dict
        if frontier_id in self.path_timers:
            del self.path_timers[frontier_id]

        # Decrement counter and check completion
        self.pending_path_count -= 1
        if self.pending_path_count == 0:
            self.select_and_navigate()

    def select_and_navigate(self):
        """Select best frontier from computed paths and navigate."""
        if not self.current_frontiers:
            return

        # Find best frontier (lowest distance)
        best_frontier = min(
            self.current_frontiers,
            key=lambda f: self.path_distances.get(id(f), float("inf")),
        )

        # Get best distance
        best_distance = self.path_distances.get(id(best_frontier), float("inf"))

        # Check if all paths failed
        if best_distance == float("inf"):
            self.get_logger().warn(
                "All path computations failed or timed out, skipping this batch"
            )
            return  # Skip batch, state will be reset at beginning of next frontiers_callback()

        # Valid path found, navigate
        if self.nav_action_client.wait_for_server(timeout_sec=0.5):
            self.send_navigation_goal(best_frontier)

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

        try:
            send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.navigation_result_callback)
            self.is_navigating = True
            self.get_logger().debug("Navigation state: NAVIGATING")
        except Exception as e:
            self.get_logger().error(f"Failed to send navigation goal: {e}")
            self.is_navigating = False

    def navigation_result_callback(self, future):
        """Handle navigation goal result.

        Args:
            future: Future containing the goal handle.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Navigation goal was rejected")
            self.is_navigating = False
            self.get_logger().debug("Navigation state: IDLE (rejected)")
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
        self.get_logger().debug(f"Navigation state: IDLE (status={status})")

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
        self.get_logger().debug(
            f"Frontiers callback: {len(msg.frontiers)} frontiers, "
            f"is_navigating={self.is_navigating}"
        )

        # Check if path computation in progress (before resetting state)
        if self.pending_path_count > 0:
            return  # Ignore new frontiers while computation in progress

        # Reset state at beginning for clean state management
        self.pending_path_count = 0
        self.path_distances = {}
        # Cancel and clear all timers
        for timer in self.path_timers.values():
            timer.cancel()
        self.path_timers = {}
        self.current_frontiers = None
        self.robot_pose = None

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

            # Get robot pose
            robot_pose = self.get_robot_pose()
            if robot_pose is None:
                return

            # Initialize path computation state
            self.current_frontiers = msg.frontiers
            self.robot_pose = robot_pose
            self.path_distances = {}
            self.pending_path_count = len(msg.frontiers)

            # Start parallel path computation for all frontiers
            for frontier in msg.frontiers:
                self.compute_path_async(frontier, robot_pose)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
