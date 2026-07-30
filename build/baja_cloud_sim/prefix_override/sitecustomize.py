import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pandafixle/Desktop/baja_cloud_sim/install/baja_cloud_sim'
