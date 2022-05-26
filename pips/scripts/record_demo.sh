#!/bin/bash
rosbag record -a -O demo.bag &
./bin/mpdm_demo
