"""
Custom service definitions for slam_robot.
"""

from slam_robot.msg import FrontierList


class GetFrontiers:
    """
    GetFrontiers service.
    ---
    slam_robot/FrontierList frontiers
    """

    class Request:
        """Empty request."""

        __slots__ = []

        def __init__(self):
            pass

    class Response:
        """Response containing frontier list."""

        __slots__ = ["frontiers"]

        def __init__(self, frontiers=None):
            if frontiers is None:
                self.frontiers = FrontierList()
            else:
                self.frontiers = frontiers
