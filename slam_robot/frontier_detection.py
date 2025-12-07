"""
Frontier detection algorithm using BFS-based exploration.
Adapted from RBE3002 FrontierSearch class.
"""

from nav_msgs.msg import OccupancyGrid
from slam_robot.msg import Frontier, FrontierList
from slam_robot.frontier_utils import (
    get_cell_value,
    grid_to_world,
    neighbors_of_4,
    neighbors_of_8,
    is_cell_in_bounds,
)

MIN_FRONTIER_SIZE = 8
WALKABLE_THRESHOLD = 50


def is_new_frontier_cell(
    mapdata: OccupancyGrid, cell: tuple[int, int], is_frontier: dict
) -> bool:
    """Check if a cell is a new frontier cell.

    A frontier cell must be:
    1. Unknown (-1)
    2. Not already marked as frontier
    3. Have at least one free neighbor (4-connected, value < 50)

    Args:
        mapdata: The occupancy grid map data.
        cell: The cell coordinate as (x, y) tuple.
        is_frontier: Dictionary tracking which cells are already frontiers.

    Returns:
        True if the cell is a new frontier cell, False otherwise.
    """
    # Cell must be unknown and not already a frontier
    if not is_cell_in_bounds(mapdata, cell):
        return False

    if get_cell_value(mapdata, cell) != -1 or cell in is_frontier:
        return False

    # Cell should have at least one connected cell that is free
    # Check neighbors that are in bounds (not necessarily walkable)
    for neighbor in neighbors_of_4(mapdata, cell, must_be_walkable=False):
        neighbor_value = get_cell_value(mapdata, neighbor)
        if neighbor_value >= 0 and neighbor_value < WALKABLE_THRESHOLD:
            return True

    return False


def build_new_frontier(
    mapdata: OccupancyGrid, initial_cell: tuple[int, int], is_frontier: dict
) -> Frontier:
    """Build a new frontier starting from an initial frontier cell.

    Uses BFS (8-connected) to find all connected frontier cells.

    Args:
        mapdata: The occupancy grid map data.
        initial_cell: The initial frontier cell coordinate.
        is_frontier: Dictionary tracking which cells are frontiers.

    Returns:
        A Frontier message with size and centroid.
    """
    # Initialize frontier fields
    size = 1
    centroid_x = initial_cell[0]
    centroid_y = initial_cell[1]

    # Create queue for breadth-first search
    queue = []
    queue.append(initial_cell)

    # Breadth-first search for frontier cells
    while queue:
        current = queue.pop(0)

        # Use 8-connected neighbors, checking bounds but not requiring walkability
        for neighbor in neighbors_of_8(mapdata, current, must_be_walkable=False):
            if is_new_frontier_cell(mapdata, neighbor, is_frontier):
                # Mark as frontier
                is_frontier[neighbor] = True

                # Update size and centroid
                size += 1
                centroid_x += neighbor[0]
                centroid_y += neighbor[1]
                queue.append(neighbor)

    # Calculate centroid by taking the average
    centroid_x /= size
    centroid_y /= size

    # Convert centroid to world coordinates
    centroid = grid_to_world(mapdata, (int(centroid_x), int(centroid_y)))

    return Frontier(size=size, centroid=centroid)


def detect_frontiers(
    mapdata: OccupancyGrid,
    start_pos: tuple[int, int],
    min_size: int = MIN_FRONTIER_SIZE,
) -> FrontierList:
    """Detect frontiers in the map using BFS-based exploration.

    Algorithm:
    1. Initialize BFS queue with start_pos (robot position in grid coordinates)
    2. Initialize visited dict and is_frontier dict
    3. While queue not empty:
       - Pop current cell
       - For each 4-connected neighbor:
         - If neighbor is free (value >= 0) and not visited: add to queue
         - If neighbor is unknown (-1) and is new frontier cell: build frontier
    4. Return FrontierList with all frontiers >= min_size

    Args:
        mapdata: The occupancy grid map data.
        start_pos: The robot position in grid coordinates as (x, y) tuple.
        min_size: Minimum frontier size to include (default: 8).

    Returns:
        A FrontierList containing all detected frontiers.
    """
    # Create queue for breadth-first search
    queue = []
    queue.append(start_pos)

    # Initialize dictionaries for keeping track of visited and frontier cells
    visited = {}
    is_frontier = {}
    visited[start_pos] = True

    # Initialize list of frontiers
    frontiers = []

    while queue:
        current = queue.pop(0)
        for neighbor in neighbors_of_4(mapdata, current):
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
