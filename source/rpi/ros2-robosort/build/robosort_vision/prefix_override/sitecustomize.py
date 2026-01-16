import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/robosort/robo-sort/source/rpi/ros2-robosort/install/robosort_vision'
