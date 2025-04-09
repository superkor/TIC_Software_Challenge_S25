#!/bin/bash
if [ -z "$1" ]; then
	echo "Usage vpn.sh ROS_DOMAIN_ID."
	echo "e.g. vpn.sh 1"
	exit 1
fi
set -e
ping -c 1 -w 10 robohub.eng.uwaterloo.ca > /dev/null || (echo -e "\e[31mCould not ping srv, check your network connections\e[39m"; exit 1)
echo "Remote reachable"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
cd $DIR

if [ -d "$DIR/../bin" ]; then
	PATH=$DIR/../bin:$PATH
fi
if [ -d "$DIR/../lib" ]; then
	LD_LIBRARY_PATH=$DIR/../lib:$LD_LIBRARY_PATH
fi

which openvpn > /dev/null || (echo "openvpn not found, installing it"; sudo apt update; sudo apt install -y openvpn)


# and using UDP 
sudo bash -c "(echo 1 > /proc/sys/net/ipv4/ipfrag_time; echo 134217728 > /proc/sys/net/ipv4/ipfrag_high_thresh ) || echo \"Cannot apply settings\""

echo "ROS_DOMAIN_ID=$1"
tmpfile=$(mktemp /tmp/turtlebot.env.XXXXXX)
echo export ROS_DOMAIN_ID=$1 > $tmpfile
# TODO: get sudo permanent
function cleanup()
{
	echo "Cleaning up"
    rm $tmpfile
}

trap "cleanup" SIGINT SIGTERM
echo "Enter your UW credentials next"
chmod go-rwx client.key
sudo LD_LIBRARY_PATH=${LD_LIBRARY_PATH} $(which openvpn) --config turtlebot-tap.conf --setenv SETTINGS $tmpfile

