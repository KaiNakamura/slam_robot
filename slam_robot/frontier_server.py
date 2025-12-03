"""
Frontier Server Node
Publishes frontier detection results on a topic.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Empty
from tf2_ros import TransformListener, Buffer
from slam_robot.frontier_detection import detect_frontiers
from slam_robot.frontier_utils import world_to_grid


class FrontierServerNode(Node):
    """Node that publishes frontier detection results."""

    def __init__(self):
        super().__init__("frontier_server")

        # Subscribers
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )

        # Subscribe to trigger topic (empty message triggers frontier detection)
        self.trigger_subscriber = self.create_subscription(
            Empty, "/get_frontiers", self.trigger_callback, 10
        )

        # Publisher for frontiers (using MarkerArray to encode frontier data)
        self.frontiers_publisher = self.create_publisher(MarkerArray, "/frontiers", 10)

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State
        self.current_map = None

        # Timer to periodically publish frontiers (1 Hz)
        self.create_timer(1.0, self.publish_frontiers)

        self.get_logger().info("Frontier server node started")

    def map_callback(self, msg: OccupancyGrid):
        """Store the latest map data."""
        self.current_map = msg
        self.get_logger().debug(f"Received map: {msg.info.width}x{msg.info.height}")

    def trigger_callback(self, msg):
        """Handle trigger to detect frontiers."""
        self.publish_frontiers()

    def publish_frontiers(self):
        """Detect and publish frontiers."""
        # Check if map available
        if self.current_map is None:
            return

        # Get robot pose via TF
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            robot_pos_world = transform.transform.translation
            robot_pos_grid = world_to_grid(self.current_map, robot_pos_world)
        except Exception as e:
            self.get_logger().debug(f"Failed to get robot pose: {e}")
            return

        # Detect frontiers
        frontier_list = detect_frontiers(self.current_map, robot_pos_grid, min_size=8)

        # Convert to MarkerArray for publishing
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
            # Store size in scale.x (we'll use scale.y for other data if needed)
            marker.scale.x = float(frontier.size)  # Store size here
            marker.scale.y = 0.2  # Visual size
            marker.scale.z = 0.2
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)

        # Publish frontiers
        self.frontiers_publisher.publish(marker_array)
        self.get_logger().info(f"Published {len(frontier_list.frontiers)} frontiers")


def main(args=None):
    rclpy.init(args=args)
    node = FrontierServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
