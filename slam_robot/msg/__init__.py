"""
Custom message definitions for slam_robot.
"""

from geometry_msgs.msg import Point
import struct


class Frontier:
    """
    Frontier message.
    uint32 size
    geometry_msgs/Point centroid
    """

    __slots__ = ["size", "centroid"]

    def __init__(self, size=0, centroid=None):
        self.size = int(size)
        if centroid is None:
            self.centroid = Point()
        else:
            self.centroid = centroid

    def __repr__(self):
        return f"Frontier(size={self.size}, centroid={self.centroid})"


class FrontierList:
    """
    FrontierList message.
    slam_robot/Frontier[] frontiers
    """

    __slots__ = ["frontiers"]

    def __init__(self, frontiers=None):
        if frontiers is None:
            self.frontiers = []
        else:
            self.frontiers = list(frontiers)

    def __repr__(self):
        return f"FrontierList(frontiers={len(self.frontiers)} frontiers)"
