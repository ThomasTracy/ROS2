import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jszr/Software/Ros2/Topic/novel/install/python_topic'
