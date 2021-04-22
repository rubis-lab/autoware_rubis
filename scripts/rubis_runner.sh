#!/bin/bash
echo "running rubis_runner"
/home/rubis/AutowareAuto/scripts/rubis_initial_pose.sh

ros2 run rubis_main rubis_main_runner
