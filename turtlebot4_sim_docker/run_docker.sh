#!/bin/bash
# Allow local root connections to the X server
xhost +local:root

docker run -it --rm --net=host \
  --name challenge \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/snd:/dev/snd \
  -v <PUT_PATH_TO_REPO_HERE>/TIC_Software_Challenge_S25:/TIC_Software_Challenge_S25 \
  -w /TIC_SoftwareChallenge_S25 \
  turtlebot4_test:latest \
  # You will need to change the path to your home directory in line 10
  # You can get the path to your TIC_Software_Challenge_S25 folder by running the command:
  # readlink -f .
  # from within this repo
bash -c "cd /simulation_files && ./install_sim_files.sh && bash"
