#!/bin/bash
set -e
#set -v

source $SETTINGS

ip link add link $dev name ${dev}.${ROS_DOMAIN_ID} type vlan id ${ROS_DOMAIN_ID}
ip link set dev ${dev}.${ROS_DOMAIN_ID} up

ip addr add dev ${dev}.${ROS_DOMAIN_ID} 192.168.186.10/24

#ros2 daemon stop
#ping -q -c 1 -w 10 192.168.186.3 > /dev/null && echo "Raspberry Pi reachable" || (echo "Failed to ping Raspberry Pi"; exit -1)
#ping -q -c 1 -w 10 192.168.186.2 > /dev/null && echo "Base reachable" || (echo "Failed to ping Base"; exit -1)
echo "Tunnel ready"
echo "Stopping this process will terminate the VPN"
echo "Don't forget to restart the ros2 daemon if necessary"

