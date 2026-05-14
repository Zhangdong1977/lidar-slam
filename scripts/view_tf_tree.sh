#!/bin/bash
# 查看 TF 树
# 用法: ./scripts/view_tf_tree.sh


set -e

# GUI 程序需要
export DISPLAY=:0
for auth in /run/user/$(id -u)/.mutter-Xwaylandauth.* /home/$(whoami)/.Xauthority; do
    if [ -f "$auth" ]; then
        export XAUTHORITY="$auth"
        break
    fi
done


eval "$(conda shell.bash hook)"
conda activate lidar_slam
source /opt/ros/jazzy/setup.bash

ros2 run rqt_tf_tree rqt_tf_tree
