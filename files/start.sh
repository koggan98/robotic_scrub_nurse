#!/bin/bash

tmux new-session -d -s ros_session

# Fensteraufteilung
tmux split-window -h
tmux split-window -v
tmux select-pane -L
tmux split-window -v

# Driver oben links
tmux select-pane -t 0
tmux send-keys "ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.12.10 launch_rviz:=false" C-m

# Loop unten links
tmux select-pane -t 1
tmux send-keys "ros2 launch tracking_pkg loop_launch.py" C-m

# MoveIt oben rechts
tmux select-pane -t 2
tmux send-keys "ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true" C-m

# Tool selection unten rechts
tmux select-pane -t 3
tmux send-keys "ros2 run tracking_pkg tool_selection.py" C-m

# Starte tmux-Session im aktiven Terminal
tmux attach-session -t ros_session
