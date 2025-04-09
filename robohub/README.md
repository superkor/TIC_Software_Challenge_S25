# UWBot - University of Waterloo Turtlebot4
This repo helps to operate the Turtlebot4 fleet. If any issue arrises, please
contact us at robohub@uwaterloo.ca .

## How can you work with the robot?
* Using a [web interface](#Web-interface) (Jupyter Notebook).
* Using a [docker container](#Docker-container).
* Using you [custom ROS2 environment](#Custom-ROS2-environment) (e.g. a VM).
Please refer to the following subsections for details.

## Web interface
Requirements: A web browser.

This uses a Jupyter notebook to program the robot.
All robots are available through 
[https://robohub.eng.uwaterloo.ca/uwbot](https://robohub.eng.uwaterloo.ca/uwbot).
This access the robot through a proxy.

Access the web interface directy (no proxy) through this url
```
https://192.168.23.XXX/uwbot-BOTID/
```
where XXX can be obtained from the display of the robot (e.g. 101)
and BOTID is the two digit number of the robot (e.g 01).

## Docker container
Requirements: A working docker installation. Test on Linux, let us know about any issues.

* Start the vpn (see Tunnel/VPN subsection)
* Start the docker
```
$ ./turtlebot4/start.sh 
access control disabled, clients can connect from any host
xhost:  must be on local machine to enable or disable access control.
Starting container: git.uwaterloo.ca:5050/robohub/turtlebot4:latest
Starting new container with name: uw_turtlebot_robohub
Container IPs: 129.97.71.29 192.168.201.23 ... 
Don't forget to set ROS_DOMAIN_ID
robohub@docker[]:~$ export ROS_DOMAIN_ID=1 
robohub@docker[1]:~$ ros2 topic list
....
```
This should list the ROS2 topics available. The list should contain e.g. `/odom` and `/scan`.

## Custom ROS2 environment
* Start the VPN, following instructions in [Tunnel](#Tunnel)
* Implement the environemnt described in the section [ROS2 middleware setup](#ROS2-middleware-setup)

## Tunnel
To operate the robot, the user must connect to the robot using a VPN.
To start the tunnel, run the script [`vpn/vpn.sh`](vpn/vpn.sh) with the
IP address of the robot as an argument.
```
./vpn/vpn.sh 192.168.23.XXX
```
Read the IP address (e.g. 192.168.23.101) from the display of the robot.

## ROS2 middleware setup
Every is done automatically, apart from setting `ROS_DOMAIN_ID`

In particular these environment variables are set
```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.fastdds.xml
```
where the file `.fastdds.xml` is copied from [`configs/.fastdds.xml`](configs/.fastdds.xml).
into the home directory.

## Multiple container instances:
Start containers with
```
CONTAINER_NAME=uwbot-01 ./start.sh
```
which now starts a container with a custom network. This way multiple openvpn
bridge connections (rendezvous_vpn) can be started on the same host with the
same ip range in the tunnel/vlan. It enables connecting to multiple robots from
one host at the same time. See below how to operate the vpn.

## Rendezvous VPN
The special rendezvous VPN uses openvpn to connect to a public VPN server
(robohub.eng.uwaterloo.ca) which removes the need for the user to be on the
same Wi-Fi as the robots. The robots also connect to this server. To start the
VPN run
```
./rendezvous_vpn/vpn.sh 2
```
to connect to `uwbot-02` and use `ROS_DOMAIN_ID=2`.

## Issues
### You cannot connect to the robot
You try to connect via the vpn script? Interpret the output:
```
$ ./vpn/vpn.sh 192.168.23.101
Remote reachable # <-- this means that pinging the robot works
Remote ready # <-- this means that the remote end of the vpn has been configured successfully
Raspberry PI reachable # <-- this means that the Raspberry PI is reachable through the tunnel
Base reachable # <-- this means that the Base is reachable though the tunnel
Tunnel ready
```

If the robot does not respond to pings, please confirm your network setup first.
Does your computer have an IP address in the right subnet? Here it's `192.168.23.0/24`.

### Base is not providing data
On the raspberry pi you tried
```
ros2 topic echo /odom
```
and no data is output. Then you can escalate
* Ping the base `ping 192.168.186.2`
* Restart the base by running `reboot_base`



