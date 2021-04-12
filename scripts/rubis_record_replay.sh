#!/bin/bash

ros2 action send_goal /planning/replaytrajectory recordreplay_planner_actions/action/ReplayTrajectory "{replay_path: "/home/rubis/record/rubis0.path"}" --feedback