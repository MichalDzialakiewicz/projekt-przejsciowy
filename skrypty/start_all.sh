#!/bin/bash

cd ~/Desktop

./start_sphinx.sh
sleep 10

./start_bebop.sh
sleep 8

./start_nodes.sh
sleep 3

rostopic pub --once /auto_tune_task std_msgs/Empty "{}"
