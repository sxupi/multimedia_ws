#!/usr/bin/env bash
docker run -it --rm   -v /dev:/dev   -v /dev/shm:/dev/shm   --privileged   --net=host   microros/micro-ros-agent:jazzy   serial --dev /dev/ttyUSB0 -b 115200 -v4