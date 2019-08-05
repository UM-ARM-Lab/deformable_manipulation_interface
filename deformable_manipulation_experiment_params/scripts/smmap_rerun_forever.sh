#!/usr/bin/env bash

while true
do
    roslaunch deformable_manipulation_experiment_params generic_experiment.launch task_type:=rope_hooks start_bullet_viewer:=false disable_all_visualizations:=true use_random_seed:=true "$@"
done
