all: image

push:
	docker push git.uwaterloo.ca:5050/robohub/turtlebot4:latest

image:
	docker pull ubuntu:jammy
	docker build --network=host -t git.uwaterloo.ca:5050/robohub/turtlebot4:latest ${CURDIR}

