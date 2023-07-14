docker build . -t ros_fastdds
docker run -it --rm -v $(pwd):/workspace -w /workspace ros_fastdds ./build.sh
