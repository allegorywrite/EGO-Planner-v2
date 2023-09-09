#!/bin/bash

cd ~/drone/EGO-Planner-v2/swarm-playground/main_ws
source devel/setup.bash

for i in {0..9}
do
    gnome-terminal -- bash -c "rosrun ego_planner drone_manager.py --drone_id $i; exec bash"
done

gnome-terminal -- bash -c "roslaunch ego_planner random_forest.launch; exec bash"