import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/robotpi/ws_lidar/install/imu_lidar_sensor_fusion'
