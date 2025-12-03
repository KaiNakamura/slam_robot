"""
Utility functions for grid operations and map manipulation.
Adapted from RBE3002 PathPlanner class.
"""

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point


def grid_to_index(mapdata: OccupancyGrid, p: tuple[int, int]) -> int:
    """
    Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.

    :param mapdata: The occupancy grid map data
    :param p: The cell coordinate as (x, y) tuple
    :return: The linear index
    """
    return p[1] * mapdata.info.width + p[0]


def get_cell_value(mapdata: OccupancyGrid, p: tuple[int, int]) -> int:
    """
    Returns the occupancy value at the given grid cell.

    :param mapdata: The occupancy grid map data
    :param p: The cell coordinate as (x, y) tuple
    :return: The cell value (-1=unknown, 0=free, 100=occupied)
    """
    return mapdata.data[grid_to_index(mapdata, p)]


def grid_to_world(mapdata: OccupancyGrid, p: tuple[int, int]) -> Point:
    """
    Transforms a cell coordinate in the occupancy grid into a world coordinate.

    :param mapdata: The occupancy grid map data
    :param p: The cell coordinate as (x, y) tuple
    :return: The position in the world as a Point
    """
    x = (p[0] + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.x
    y = (p[1] + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.y
    return Point(x=x, y=y, z=0.0)


def world_to_grid(mapdata: OccupancyGrid, wp: Point) -> tuple[int, int]:
    """
    Transforms a world coordinate into a cell coordinate in the occupancy grid.

    :param mapdata: The occupancy grid map data
    :param wp: The world coordinate as a Point
    :return: The cell position as (x, y) tuple
    """
    x = int((wp.x - mapdata.info.origin.position.x) / mapdata.info.resolution)
    y = int((wp.y - mapdata.info.origin.position.y) / mapdata.info.resolution)
    return (x, y)


def is_cell_in_bounds(mapdata: OccupancyGrid, p: tuple[int, int]) -> bool:
    """
    Check if a grid cell is within the map bounds.

    :param mapdata: The occupancy grid map data
    :param p: The cell coordinate as (x, y) tuple
    :return: True if the cell is within bounds, False otherwise
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


def is_cell_walkable(mapdata: OccupancyGrid, p: tuple[int, int]) -> bool:
    """
    Check if a cell is free and walkable.
    A cell is walkable if it is within bounds and has value < 50.

    :param mapdata: The occupancy grid map data
    :param p: The cell coordinate as (x, y) tuple
    :return: True if the cell is walkable, False otherwise
    """
    if not is_cell_in_bounds(mapdata, p):
        return False

    WALKABLE_THRESHOLD = 50
    return get_cell_value(mapdata, p) < WALKABLE_THRESHOLD


def neighbors_of_4(
    mapdata: OccupancyGrid, p: tuple[int, int], must_be_walkable: bool = True
) -> list[tuple[int, int]]:
    """
    Get 4-connected neighbors (up, down, left, right).

    :param mapdata: The occupancy grid map data
    :param p: The cell coordinate as (x, y) tuple
    :param must_be_walkable: If True, only return walkable neighbors. If False, return all in-bounds neighbors.
    :return: List of neighbor coordinates
    """
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    neighbors = []
    for direction in directions:
        candidate = (p[0] + direction[0], p[1] + direction[1])
        if must_be_walkable:
            if is_cell_walkable(mapdata, candidate):
                neighbors.append(candidate)
        else:
            if is_cell_in_bounds(mapdata, candidate):
                neighbors.append(candidate)
    return neighbors


def neighbors_of_8(
    mapdata: OccupancyGrid, p: tuple[int, int], must_be_walkable: bool = True
) -> list[tuple[int, int]]:
    """
    Get 8-connected neighbors (includes diagonals).

    :param mapdata: The occupancy grid map data
    :param p: The cell coordinate as (x, y) tuple
    :param must_be_walkable: If True, only return walkable neighbors. If False, return all in-bounds neighbors.
    :return: List of neighbor coordinates
    """
    directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    neighbors = []
    for direction in directions:
        candidate = (p[0] + direction[0], p[1] + direction[1])
        if must_be_walkable:
            if is_cell_walkable(mapdata, candidate):
                neighbors.append(candidate)
        else:
            if is_cell_in_bounds(mapdata, candidate):
                neighbors.append(candidate)
    return neighbors
