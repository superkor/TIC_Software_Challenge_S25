#!/bin/bash
sudo ip link add bridge type bridge
sudo ip link set up bridge
sudo ip link set usb0 master bridge
sudo ip link set tap0 master bridge
sudo ip link set usb0 promisc on
sudo ip link set tap0 promisc on
sudo ip addr del dev usb0 192.168.186.3/24
sudo ip addr del dev tap0 192.168.186.9/24
sudo ip addr add dev bridge 192.168.186.3/24

