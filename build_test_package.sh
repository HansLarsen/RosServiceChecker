docker build . -t ros_fastdds
docker run -it --rm -v $(pwd):/workspace -w /workspace ros_fastdds \
 bash -c "source fastdds_ws/install/setup.bash; source ros_ws/install/setup.bash; cd ros_ws; colcon build --symlink-install --packages-select test_package --cmake-args -DFASTDDS_STATISTICS=ON"