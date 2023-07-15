docker run -it -v $(pwd):/workspace \
 -w /workspace \
 --rm \
 ros_fastdds \
 bash -c "source fastdds_ws/install/setup.bash; source ros_ws/install/setup.bash; ros2 launch test_package launch_sub.py"
