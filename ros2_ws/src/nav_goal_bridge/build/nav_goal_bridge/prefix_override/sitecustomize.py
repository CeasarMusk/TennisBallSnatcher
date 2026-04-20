import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ceasarmusk/ros2_ws/src/nav_goal_bridge/install/nav_goal_bridge'
