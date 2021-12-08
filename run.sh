#!/bin/bash

gnome-terminal -t "other" -x bash -c "roslaunch nmea_navsat_driver other.launch;exec bash"

sleep 3s

gnome-terminal -t "gps" -x bash -c "roslaunch nmea_navsat_driver demo.launch;exec bash"

sleep 3s

gnome-terminal -t "rosbag_record" -x bash -c "rosbag record /tf /fix /imu/data /odom /rslidar_points;exec bash"






