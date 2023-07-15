#!/bin/bash
docker run -it -v $(pwd):/workspace \
 -w /workspace \
 -v /tmp/.X11-unix:/tmp/.X11-unix \
 -v /dev/shm:/dev/shm \
 --ipc=host \
 --pid=host \
 -e DISPLAY=:0 \
 -e RUNLEVEL=3 \
 -e NVIDIA_DRIVER_CAPABILITIES=all \
 --gpus all \
 -e XDG_RUNTIME_DIR=/tmp/runtime-root \
 --rm \
 ros_fastdds \
 bash -c "source fastdds_ws/install/setup.bash; source ros_ws/install/setup.bash; fastdds_monitor"