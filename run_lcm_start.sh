#!/bin/bash

# enable multicast and add route for lcm out the top
sudo ifconfig enP7p1s0 multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev enP7p1s0

# If u want to set other network
# change below 
# enP7p1s0 -> other network (check by ip a in terminal)
# 224.0.0.0 -> must be same as server/client netmask (manual setting)
# After u type down .sh file, u need to give permission for the file
# chmod +x run_lcm_start.sh 
# In general, write down chmod +x YOUR_FILE_NAME.sh