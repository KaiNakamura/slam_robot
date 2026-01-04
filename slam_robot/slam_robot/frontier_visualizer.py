import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, GridCells
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Quaternion
from slam_robot_interfaces.msg import FrontierList


class FrontierVisualizerNode(Node):
    """Node for frontier visualization in RViz."""

    def __init__(self):
        super().__init__("frontier_visualizer")

        # Subscribers
        self.frontiers_sub = self.create_subscription(
            FrontierList, "/frontiers", self.frontiers_callback
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback
        )

        # Publishers
        self.centroids_pub = self.create_publisher(MarkerArray, "/frontier_centroids")
        self.cells_pub = self.create_publisher(GridCells, "/frontier_cells")

        # State
        self.map_resolution = None

        self.get_logger().info("Frontier visualizer node started")

    def map_callback(self, msg: OccupancyGrid):
        """Cache map resolution for GridCells."""
        self.map_resolution = msg.info.resolution

    def frontiers_callback(self, msg: FrontierList):
        """Handle incoming frontiers and publish visualizations."""
        if self.map_resolution is None:
            self.get_logger().debug("Waiting for map resolution")
            return

        self.publish_centroids(msg)
        self.publish_cells(msg)

    def publish_centroids(self, frontier_list: FrontierList):
        """Publish MarkerArray for frontier centroids.

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
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
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

        self.centroids_pub.publish(marker_array)

    def publish_cells(self, frontier_list: FrontierList):
        """Publish GridCells for all frontier cells.

        Args:
            frontier_list: List of frontiers to visualize.
        """
        grid_cells = GridCells()
        grid_cells.header.frame_id = "map"
        grid_cells.header.stamp = self.get_clock().now().to_msg()
        grid_cells.cell_width = self.map_resolution
        grid_cells.cell_height = self.map_resolution

        for frontier in frontier_list.frontiers:
            grid_cells.cells.extend(frontier.cells)

        self.cells_pub.publish(grid_cells)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierVisualizerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
