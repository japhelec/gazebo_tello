#!/bin/bash

rostopic pub  /tello/vel_cmd geometry_msgs/Twist "{linear:  {x: $1, y: $2, z: $3}, angular: {x: 0,y: 0,z: $4}}"
exit 0