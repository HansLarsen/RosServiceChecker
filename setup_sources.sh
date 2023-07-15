#!/bin/bash
docker build . -t ros_fastdds
docker run -it --rm -v $(pwd):/workspace -w /workspace ros_fastdds bash -c "cd fastdds_ws; vcs import src < fastrtps.repos"
docker run -it --rm -v $(pwd):/workspace -w /workspace ros_fastdds bash -c "cd ros_ws; vcs import src < ros2.repos"