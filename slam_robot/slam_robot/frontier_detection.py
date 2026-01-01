"""
Frontier detection algorithm using BFS-based exploration.
"""

from collections import deque

from nav_msgs.msg import OccupancyGrid
from slam_robot_interfaces.msg import Frontier, FrontierList
from slam_robot.frontier_utils import (
    FREE_THRESHOLD,
    get_cell_value,
    grid_to_world,
    get_neighbors_of_4,
    get_neighbors_of_8,
    is_cell_in_bounds,
)

MIN_FRONTIER_SIZE = 8


def is_new_frontier_cell(
    mapdata: OccupancyGrid, cell: tuple[int, int], is_frontier: dict
) -> bool:
    """Check if a cell is a new frontier cell.

    Args:
        mapdata: The occupancy grid map data.
        cell: The cell coordinate as (x, y) tuple.
        is_frontier: Dictionary tracking which cells are already frontiers.

    Returns:
        True if the cell is a new frontier cell, False otherwise.
    """
    # Cell must be in bounds
    if not is_cell_in_bounds(mapdata, cell):
        return False

    # Cell must be unknown and not already a frontier
    if get_cell_value(mapdata, cell) != -1 or cell in is_frontier:
        return False

    # Cell should have at least one connected cell that is free
    # Check neighbors that are in bounds (not necessarily free)
    for neighbor in get_neighbors_of_4(mapdata, cell, must_be_free=False):
        neighbor_value = get_cell_value(mapdata, neighbor)
        if neighbor_value >= 0 and neighbor_value < FREE_THRESHOLD:
            return True

    return False


def build_new_frontier(
    mapdata: OccupancyGrid, initial_cell: tuple[int, int], is_frontier: dict
) -> Frontier:
    """Build a new frontier starting from an initial frontier cell.

    Args:
        mapdata: The occupancy grid map data.
        initial_cell: The initial frontier cell coordinate.
        is_frontier: Dictionary tracking which cells are frontiers.

    Returns:
        A Frontier message.
    """
    # Initialize frontier fields
    size = 1
    centroid_x = initial_cell[0]
    centroid_y = initial_cell[1]

    # Collect cell world coordinates
    cells = [grid_to_world(mapdata, initial_cell)]

    # Create queue for breadth-first search
    queue = deque()
    queue.append(initial_cell)

    # Breadth-first search for frontier cells
    while queue:
        current = queue.popleft()

        # Use 8-connected neighbors, checking bounds but not requiring free space
        for neighbor in get_neighbors_of_8(mapdata, current, must_be_free=False):
            if is_new_frontier_cell(mapdata, neighbor, is_frontier):
                # Mark as frontier
                is_frontier[neighbor] = True

                # Update size and centroid
                size += 1
                centroid_x += neighbor[0]
                centroid_y += neighbor[1]

                # Append cell
                cells.append(grid_to_world(mapdata, neighbor))

                queue.append(neighbor)

    # Calculate centroid by taking the average
    centroid_x /= size
    centroid_y /= size

    # Convert centroid to world coordinates
    centroid = grid_to_world(mapdata, (int(centroid_x), int(centroid_y)))

    return Frontier(size=size, centroid=centroid, cells=cells)


def detect_frontiers(
    mapdata: OccupancyGrid,
    start_pos: tuple[int, int],
    min_size: int = MIN_FRONTIER_SIZE,
) -> FrontierList:
    """Detect frontiers in the map using BFS-based exploration.

    Args:
        mapdata: The occupancy grid map data.
        start_pos: The robot position in grid coordinates as (x, y) tuple.
        min_size: Minimum frontier size to include.

    Returns:
        A FrontierList containing all detected frontiers meeting the size threshold.
    """
    # Create queue for breadth-first search
    queue = deque()
    queue.append(start_pos)

    # Initialize dictionaries for keeping track of visited and frontier cells
    visited = {}
    is_frontier = {}
    visited[start_pos] = True

    # Initialize list of frontiers
    frontiers = []

    while queue:
        current = queue.popleft()
        for neighbor in get_neighbors_of_4(mapdata, current):
            neighbor_value = get_cell_value(mapdata, neighbor)
            if neighbor_value >= 0 and neighbor not in visited:
                visited[neighbor] = True
                queue.append(neighbor)
            elif is_new_frontier_cell(mapdata, neighbor, is_frontier):
                # Mark as frontier
                is_frontier[neighbor] = True

                # Build new frontier
                new_frontier = build_new_frontier(mapdata, neighbor, is_frontier)
                if new_frontier.size >= min_size:
                    frontiers.append(new_frontier)

    return FrontierList(frontiers=frontiers)
