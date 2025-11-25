import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kai/ros2_ws/src/slam_robot/install/slam_robot'
