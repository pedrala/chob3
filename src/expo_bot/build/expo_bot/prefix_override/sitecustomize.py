import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/viator/ws/chob3_ws/src/expo_bot/install/expo_bot'
