import sys
if sys.prefix == '/Users/wiktorkowalczyk/miniconda3/envs/ros_env':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/wiktorkowalczyk/Desktop/School/FYP/FinalYearProject/install/helper_nodes'
