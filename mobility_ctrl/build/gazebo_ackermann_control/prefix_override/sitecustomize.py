import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/brittc/Exodus2025/mobility_ctrl/install/gazebo_ackermann_control'
