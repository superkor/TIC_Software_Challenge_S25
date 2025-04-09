ARG BASE_IMAGE=ubuntu
ARG BASE_TAG=jammy
ARG BASE=${BASE_IMAGE}:${BASE_TAG}
FROM ${BASE}

# do not query user during built, should be unset later
ENV DEBIAN_FRONTEND=noninteractive

# install basic set of utilities required for docker_entrypoint.sh
RUN DEBIAN_FRONTEND=noninteractive DEBCONF_NONINTERACTIVE_SEEN=true apt-get update && apt-get install -q -y --no-install-recommends \
        sudo \
        iproute2 \
        apt-transport-https tzdata \
    && rm -rf /var/lib/apt/lists/*

# set timezone
RUN echo "America/Toronto" > /etc/timezone && dpkg-reconfigure -f noninteractive tzdata

# use local mirror for ubuntu packages
# if ubuntu mainline gets too unstable this can be switched
RUN sed -E -i "s/http:\/\/.*\.ubuntu\.com/http:\/\/mirror.csclub.uwaterloo.ca/" /etc/apt/sources.list

# set default password (make sure ssh has root login disable)
RUN echo "root:root" | chpasswd

# install a set of utilities
RUN ln -fs /usr/share/zoneinfo/America/Toronto /etc/localtime
RUN DEBIAN_FRONTEND=noninteractive DEBCONF_NONINTERACTIVE_SEEN=true apt-get update && \
    apt-get install -q -y --no-install-recommends iputils-ping dnsutils vim-gtk sudo x11-xkb-utils xfce4-session build-essential usbutils less openssh-client nmap strace bash-completion zsh unzip wget sshfs cmake-curses-gui lsof ltrace lsof screen git gitk git-gui && \
    rm -rf /var/lib/apt/lists/*

# install more tools
RUN DEBIAN_FRONTEND=noninteractive DEBCONF_NONINTERACTIVE_SEEN=true apt-get update && \
    apt-get install -q -y --no-install-recommends python3-pip python3-scipy python3-matplotlib && \
    rm -rf /var/lib/apt/lists/*

# fix starting QT applications in container (TODO: why do we need this?)
RUN echo QT_X11_NO_MITSHM=1 >> /etc/environment

# setup entrypoint
COPY ./docker_entrypoint.sh /root

ENTRYPOINT ["/root/docker_entrypoint.sh"]

# install nvidia stuff in all images, should be bypassed on non-nvidia platforms
#from
# - https://gitlab.com/nvidia/container-images/opengl/-/blob/ubuntu18.04/base/Dockerfile
# - https://gitlab.com/nvidia/container-images/opengl/-/blob/ubuntu18.04/glvnd/runtime/Dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
        libxau6 \
        libxdmcp6 \
        libxcb1 \
        libxext6 \
        libx11-6 \
    	libglvnd0 \
	libgl1 \
	libglx0 \
	libegl1 \
	libgles2 && \
    rm -rf /var/lib/apt/lists/*

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
        ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
        ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compat32,utility

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

# Required for non-glvnd setups.
ENV LD_LIBRARY_PATH /usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}:/usr/local/nvidia/lib:/usr/local/nvidia/lib64

COPY 10_nvidia.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# TODO: install CUDA development stuff

# install nvidia stuff in all images, should be bypassed on non-nvidia platforms
#from
# - https://gitlab.com/nvidia/container-images/opengl/-/blob/ubuntu18.04/base/Dockerfile
# - https://gitlab.com/nvidia/container-images/opengl/-/blob/ubuntu18.04/glvnd/runtime/Dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
        libxau6 \
        libxdmcp6 \
        libxcb1 \
        libxext6 \
        libx11-6 \
    	libglvnd0 \
	libgl1 \
	libglx0 \
	libegl1 \
	libgles2 && \
    rm -rf /var/lib/apt/lists/*

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
        ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
        ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compat32,utility

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

# Required for non-glvnd setups.
ENV LD_LIBRARY_PATH /usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}:/usr/local/nvidia/lib:/usr/local/nvidia/lib64

COPY 10_nvidia.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# install prequisites from ros installation
RUN apt-get update && apt-get install -q -y \
    apt-utils \
    dirmngr \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# setup keys: ROS
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

## setup sources.list for ROS: using mirror on robotlab10
# if ros mainline gets too unstable this can be this mirror should be frozen

#RUN /bin/echo -e "deb http://packages.ros.org/ros/ubuntu/ $(lsb_release -s -c) main\n" > /etc/apt/sources.list.d/ros.list
RUN /bin/echo -e "deb http://robohub.eng.uwaterloo.ca/mirror/ros/ros2/ubuntu/ $(lsb_release -s -c) main\n" > /etc/apt/sources.list.d/ros.list

## install bootstrap tools
RUN apt-get update && apt-key update && apt-get install --no-install-recommends -y \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO=${ROS_DISTRO}
# install ros packages
RUN apt-get update && apt-get install -q -y \
    ros-humble-ros-core && \
    rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
#RUN rosdep init \
#    && rosdep update \
#    && rosdep fix-permissions

# install more robot dependencies
RUN apt-get update && \
    apt-get install -q -y ros-humble-turtlebot4-base ros-humble-turtlebot4-bringup ros-humble-turtlebot4-cpp-tutorials ros-humble-turtlebot4-description ros-humble-turtlebot4-desktop ros-humble-turtlebot4-diagnostics ros-humble-turtlebot4-msgs ros-humble-turtlebot4-navigation ros-humble-turtlebot4-node ros-humble-turtlebot4-python-tutorials ros-humble-turtlebot4-robot ros-humble-turtlebot4-tests ros-humble-turtlebot4-tutorials ros-humble-turtlebot4-viz htop psmisc ros-humble-irobot-create-ignition-bringup mesa-utils mtr-tiny python3-catkin python3-colcon* ros-humble-apriltag ros-humble-demo-nodes-cpp ros-humble-rmw-fastrtps-cpp ros-humble-rosbag2 ros-humble-teleop-twist-keyboard ros-humble-tf2-tools tcpdump openvpn ros-humble-compressed-image-transport ros-humble-rqt-image-view && \
    rm -rf /var/lib/apt/lists/*
RUN apt-get update

