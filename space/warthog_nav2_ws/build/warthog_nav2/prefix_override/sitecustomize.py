import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jotheesh/warthog_nav2_ws/install/warthog_nav2'
