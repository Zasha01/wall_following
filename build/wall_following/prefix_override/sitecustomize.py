import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ptratsae/robotics/assignment_1/wall_following/install/wall_following'
