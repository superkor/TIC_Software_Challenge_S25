#!/bin/bash
# Allow local root connections to the X server
xhost +local:root

docker run -it --rm --net=host \
  --name challenge \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/snd:/dev/snd \
  # Your will need to change the path to your home directory in the following three lines
  # You can get the path to your TIC_Software_Challenge_S25 folder by running the command:
  # readlink -f .
  # from withing this repo
  -v /home/ideasclinic/Documents:/home/ideasclinic/Documents \ 
  -v /home/ideasclinic/Documents/challenge_toyota/TMMC-Working/simulation_files:/simulation_files \
  -w /home/ideasclinic/Documents \
  turtlebot4_test:latest \
  bash -c "cd /simulation_files && ./install_sim_files.sh && bash"
