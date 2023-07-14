docker run -it -v $(pwd):/workspace \
 -w /workspace \
 -v /tmp/.X11-unix:/tmp/.X11-unix \
 -v /dev/shm:/dev/shm \
 --ipc=host \
 --pid=host \
 --rm \
 --cpus=0.05 \
 --memory=0.5 \
 ros_fastdds \
 bash -c "source fastdds_ws/install/setup.bash; source ros_ws/install/setup.bash; ros2 launch test_package launch_pub.py"
