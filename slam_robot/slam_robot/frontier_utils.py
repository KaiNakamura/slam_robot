"""
Utility functions for grid operations and map manipulation.
"""

import math

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from slam_robot_interfaces.msg import Frontier


def grid_to_index(mapdata: OccupancyGrid, p: tuple[int, int]) -> int:
    """Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.

    Args:
        mapdata: The occupancy grid map data.
        p: The cell coordinate as (x, y) tuple.

    Returns:
        The linear index.
    """
    return p[1] * mapdata.info.width + p[0]


def get_cell_value(mapdata: OccupancyGrid, p: tuple[int, int]) -> int:
    """Returns the occupancy value at the given grid cell.

    Args:
        mapdata: The occupancy grid map data.
        p: The cell coordinate as (x, y) tuple.

    Returns:
        The cell value (-1=unknown, 0=free, 100=occupied).
    """
    return mapdata.data[grid_to_index(mapdata, p)]


def grid_to_world(mapdata: OccupancyGrid, p: tuple[int, int]) -> Point:
    """Transforms a cell coordinate in the occupancy grid into a world coordinate.

    Args:
        mapdata: The occupancy grid map data.
        p: The cell coordinate as (x, y) tuple.

    Returns:
        The position in the world as a Point.
    """
    x = (p[0] + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.x
    y = (p[1] + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.y
    return Point(x=x, y=y, z=0.0)


def world_to_grid(mapdata: OccupancyGrid, wp: Point) -> tuple[int, int]:
    """Transforms a world coordinate into a cell coordinate in the occupancy grid.

    Args:
        mapdata: The occupancy grid map data.
        wp: The world coordinate as a Point.

    Returns:
        The cell position as (x, y) tuple.
    """
    x = int((wp.x - mapdata.info.origin.position.x) / mapdata.info.resolution)
    y = int((wp.y - mapdata.info.origin.position.y) / mapdata.info.resolution)
    return (x, y)


def is_cell_in_bounds(mapdata: OccupancyGrid, p: tuple[int, int]) -> bool:
    """Check if a grid cell is within the map bounds.

    Args:
        mapdata: The occupancy grid map data.
        p: The cell coordinate as (x, y) tuple.

    Returns:
        True if the cell is within bounds, False otherwise.
    """
    width = mapdata.info.width
    height = mapdata.info.height
    x = p[0]
    y = p[1]

    if x < 0 or x >= width:
        return False
    if y < 0 or y >= height:
        return False
    return True


def is_cell_free(
    mapdata: OccupancyGrid, p: tuple[int, int], free_threshold: int
) -> bool:
    """Check if a cell is free.

    A cell is free if it is within bounds and has value < free_threshold.

    Args:
        mapdata: The occupancy grid map data.
        p: The cell coordinate as (x, y) tuple.
        free_threshold: Occupancy value threshold for free space (0-100).

    Returns:
        True if the cell is free, False otherwise.
    """
    if not is_cell_in_bounds(mapdata, p):
        return False

    return get_cell_value(mapdata, p) < free_threshold


def get_neighbors(
    mapdata: OccupancyGrid,
    p: tuple[int, int],
    directions: list[tuple[int, int]],
    must_be_free: bool = True,
    free_threshold: int = None,
) -> list[tuple[int, int]]:
    """Helper for getting neighbors with given directions.

    Args:
        mapdata: The occupancy grid map data.
        p: The cell coordinate as (x, y) tuple.
        directions: List of direction offsets as (dx, dy) tuples.
        must_be_free: If True, only return free neighbors. If False, return all in-bounds neighbors.
        free_threshold: Occupancy value threshold for free space (0-100). Required if must_be_free=True.

    Returns:
        List of neighbor coordinates.
    """
    neighbors = []
    for direction in directions:
        candidate = (p[0] + direction[0], p[1] + direction[1])
        if must_be_free:
            if free_threshold is None:
                raise ValueError(
                    "free_threshold must be provided when must_be_free=True"
                )
            if is_cell_free(mapdata, candidate, free_threshold):
                neighbors.append(candidate)
        else:
            if is_cell_in_bounds(mapdata, candidate):
                neighbors.append(candidate)
    return neighbors


def get_neighbors_of_4(
    mapdata: OccupancyGrid,
    p: tuple[int, int],
    must_be_free: bool = True,
    free_threshold: int = None,
) -> list[tuple[int, int]]:
    """Get 4-connected neighbors (up, down, left, right).

    Args:
        mapdata: The occupancy grid map data.
        p: The cell coordinate as (x, y) tuple.
        must_be_free: If True, only return free neighbors. If False, return all in-bounds neighbors.
        free_threshold: Occupancy value threshold for free space (0-100). Required if must_be_free=True.

    Returns:
        List of neighbor coordinates.
    """
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    return get_neighbors(mapdata, p, directions, must_be_free, free_threshold)


def get_neighbors_of_8(
    mapdata: OccupancyGrid,
    p: tuple[int, int],
    must_be_free: bool = True,
    free_threshold: int = None,
) -> list[tuple[int, int]]:
    """Get 8-connected neighbors (includes diagonals).

    Args:
        mapdata: The occupancy grid map data.
        p: The cell coordinate as (x, y) tuple.
        must_be_free: If True, only return free neighbors. If False, return all in-bounds neighbors.
        free_threshold: Occupancy value threshold for free space (0-100). Required if must_be_free=True.

    Returns:
        List of neighbor coordinates.
    """
    directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    return get_neighbors(mapdata, p, directions, must_be_free, free_threshold)


def snap_centroid_to_frontier_cell(frontier: Frontier) -> Frontier:
    """Snap centroid to the nearest frontier cell.

    This ensures the centroid is always an actual frontier cell, which helps
    avoid edge cases where the centroid ends up far away from the frontier.

    Args:
        frontier: The frontier to adjust.

    Returns:
        The frontier with adjusted centroid (modified in place).
    """
    if not frontier.cells:
        return frontier

    # Store original centroid
    original_centroid = frontier.centroid

    # Find frontier cell closest to original centroid
    min_dist = float("inf")
    nearest_cell = frontier.cells[0]

    for cell in frontier.cells:
        dx = cell.x - original_centroid.x
        dy = cell.y - original_centroid.y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < min_dist:
            min_dist = dist
            nearest_cell = cell

    frontier.centroid = nearest_cell
    return frontier
