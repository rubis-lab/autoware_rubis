#!/bin/bash

ros2 action send_goal /planning/recordtrajectory recordreplay_planner_actions/action/RecordTrajectory "{record_path: "/home/rubis/record/rubis0.path"}" --feedback