#!/bin/bash
# Allow local root connections to the X server
xhost +local:root

docker run -it --rm --net=host \
  --cap-add=NET_ADMIN --device=/dev/net/tun \
  --name challenge \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/snd:/dev/snd \
  -v /home/jack/TIC_Software_Challenge_S25:/TIC_Software_Challenge_S25 \
  -w /TIC_Software_Challenge_S25 \
  turtlebot4_test:latest \
  # You will need to change the path to your home directory in line 10
  # You can get the path to your TIC_Software_Challenge_S25 folder by running the command:
  # readlink -f .
  # from within this repo
bash -c "cd /simulation_files && chmod +x install_sim_files.sh && ./install_sim_files.sh && bash"