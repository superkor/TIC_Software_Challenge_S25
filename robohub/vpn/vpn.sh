#!/bin/bash
if [ -z "$1" ]; then
	echo "Usage vpn.sh ROBOT_IP (from display)"
	exit 1
fi
set -e
ping -c 1 -w 10 $1 > /dev/null || (echo -e "\e[31mCould not ping robot, check your network connections\e[39m"; exit 1)
echo "Remote reachable"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
cd $DIR
SSH_KEY=$DIR/../configs/.ssh/id_rsa_robohub
chmod go-rw ${SSH_KEY}

# find own IP on turtlebot network
if [ -z ${TB_NET} ]; then
	TB_NET=192.168.23
fi

NETWORK_FOUND=0
for ip in $(hostname --all-ip-addresses); do
  if [[ $ip == ${TB_NET}.* ]]; then
    export LOCAL_IP=$ip
    NETWORK_FOUND=1
  fi
done
if [[ "$NETWORK_FOUND" == "0" ]]; then
    echo "Not on turtlebot network!"
    exit 1
fi

REMOTE_IP=$1
# remove any existing vpn on the robot
ssh -i $SSH_KEY -o StrictHostKeyChecking=no ubuntu@${REMOTE_IP} "sudo ip link del gretap1 2> /dev/null; sudo modprobe fou; sudo ip fou add port 52323 gue; sudo ip link add name gretap1 type gretap local ${REMOTE_IP} remote ${LOCAL_IP} ignore-df nopmtudisc encap gue encap-sport auto encap-dport 52323 && sudo ip add add 192.168.186.11/24 dev gretap1 && sudo ip link set gretap1 mtu 1500 && sudo ip link set gretap1 master bridge && sudo ip link set gretap1 promisc on&& sudo ip link set up gretap1 && echo Remote ready"

# set up local vpn
sudo ip link del gretap1 2> /dev/null || /bin/true
sudo modprobe fou
sudo ip fou del port 52323 gue 2> /dev/null || /bin/true
sudo ip fou add port 52323 gue
sudo ip link add name gretap1 type gretap local ${LOCAL_IP} remote ${REMOTE_IP} ignore-df nopmtudisc encap gue encap-sport auto encap-dport 52323
sudo ip link set gretap1 mtu 1500
sudo ip addr add 192.168.186.10/24 dev gretap1
sudo ip link set up gretap1

# Apply network settings to make DDS work over non-perfect networks
# This fixed in particular issues when subscribing large message (images)
# and using UDP 
sudo bash -c "echo 1 > /proc/sys/net/ipv4/ipfrag_time"
sudo bash -c "echo 134217728  > /proc/sys/net/ipv4/ipfrag_high_thresh"

# TODO: get sudo permanent
function cleanup()
{
	echo "Cleaning up"
	sudo ip link del gretap1
	ssh -i $SSH_KEY -o StrictHostKeyChecking=no ubuntu@${REMOTE_IP} sudo ip link del gretap1
}

trap "cleanup" SIGINT SIGTERM

ping -q -c 1 -w 10 192.168.186.3 > /dev/null && echo "Raspberry Pi reachable" || (echo "Failed to ping Raspberry Pi"; exit -1)
ping -q -c 1 -w 10 192.168.186.2 > /dev/null && echo "Base reachable" || (echo "Failed to ping Base"; exit -1)
echo "Tunnel ready"
# call something which lingers
sleep 36000


