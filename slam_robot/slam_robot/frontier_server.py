import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformListener, Buffer
from slam_robot_interfaces.srv import GetFrontiers
from slam_robot_interfaces.msg import FrontierList
from slam_robot.frontier_detection import MIN_FRONTIER_SIZE, detect_frontiers
from slam_robot.frontier_utils import world_to_grid


class FrontierServerNode(Node):
    """Node that publishes frontier detection results."""

    def __init__(self):
        super().__init__("frontier_server")

        # Subscribers
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )

        # Service server for frontier detection
        self.get_frontiers_service = self.create_service(
            GetFrontiers, "/get_frontiers", self.get_frontiers_callback
        )

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State
        self.current_map = None

        self.get_logger().info("Frontier server node started")

    def map_callback(self, msg: OccupancyGrid):
        """Store the latest map data."""
        self.current_map = msg
        self.get_logger().debug(f"Received map: {msg.info.width}x{msg.info.height}")

    def get_frontiers_callback(self, request, response):
        """Handle GetFrontiers service request."""
        # Check if map available
        if self.current_map is None:
            response.frontiers = FrontierList()
            return response

        # Get robot pose via TF
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            robot_pos_world = transform.transform.translation
            robot_pos_grid = world_to_grid(self.current_map, robot_pos_world)
        except Exception as e:
            self.get_logger().debug(f"Failed to get robot pose: {e}")
            response.frontiers = FrontierList()
            return response

        # Detect frontiers
        frontier_list = detect_frontiers(
            self.current_map, robot_pos_grid, min_size=MIN_FRONTIER_SIZE
        )

        # Return frontiers in response
        response.frontiers = frontier_list
        self.get_logger().info(
            f"Service returning {len(frontier_list.frontiers)} frontiers"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FrontierServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
