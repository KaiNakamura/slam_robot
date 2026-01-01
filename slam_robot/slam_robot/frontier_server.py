import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformListener, Buffer
from slam_robot_interfaces.msg import FrontierList
from slam_robot.frontier_detection import MIN_FRONTIER_SIZE, detect_frontiers
from slam_robot.frontier_utils import world_to_grid


class FrontierServerNode(Node):
    """Node that publishes frontiers on map updates."""

    def __init__(self):
        super().__init__("frontier_server")

        # Subscribers
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )

        # Publishers
        self.frontiers_publisher = self.create_publisher(FrontierList, "/frontiers", 10)

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Frontier server node started")

    def map_callback(self, msg: OccupancyGrid):
        """On map updates, detect frontiers and publish."""
        self.get_logger().debug(f"Received map: {msg.info.width}x{msg.info.height}")

        # Get robot pose via TF
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            robot_pos_world = transform.transform.translation
            robot_pos_grid = world_to_grid(msg, robot_pos_world)
        except Exception as e:
            self.get_logger().debug(f"Failed to get robot pose: {e}")
            return

        # Detect frontiers
        frontier_list = detect_frontiers(
            msg, robot_pos_grid, min_size=MIN_FRONTIER_SIZE
        )

        # Publish frontiers
        self.frontiers_publisher.publish(frontier_list)
        self.get_logger().debug(f"Published {len(frontier_list.frontiers)} frontiers")


def main(args=None):
    rclpy.init(args=args)
    node = FrontierServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
