#!/bin/bash
cd fastdds_ws
colcon build --cmake-args -DFASTDDS_STATISTICS=ON
source install/setup.bash
cd ../ros_ws
colcon build --symlink-install --cmake-args -DFASTDDS_STATISTICS=ON