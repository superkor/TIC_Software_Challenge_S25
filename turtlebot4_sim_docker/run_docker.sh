#!/bin/bash
# Allow local root connections to the X server
xhost +local:root

docker run -it --rm --net=host \
  --name challenge \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/snd:/dev/snd \
  -v /home/ideasclinic/Documents:/home/ideasclinic/Documents \
  -v /home/ideasclinic/Documents/challenge_toyota/TMMC-Working/simulation_files:/simulation_files \
  -w /home/ideasclinic/Documents \
  turtlebot4-image \
  bash -c "cd /simulation_files && ./install_sim_files.sh && bash"
