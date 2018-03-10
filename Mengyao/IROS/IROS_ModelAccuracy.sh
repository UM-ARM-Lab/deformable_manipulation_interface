#!/usr/bin/env bash

mkdir -p ~/Dropbox/catkin_ws/src/smmap/logs/rope_table_linear_plus_x/model_accuracy
mkdir -p ~/Dropbox/catkin_ws/src/smmap/logs/rope_table_linear_minus_x/model_accuracy
mkdir -p ~/Dropbox/catkin_ws/src/smmap/logs/rope_table_linear_plus_y/model_accuracy
mkdir -p ~/Dropbox/catkin_ws/src/smmap/logs/rope_table_linear_minus_y/model_accuracy
mkdir -p ~/Dropbox/catkin_ws/src/smmap/logs/rope_table_linear_plus_z/model_accuracy
mkdir -p ~/Dropbox/catkin_ws/src/smmap/logs/rope_table_linear_pull_down_into_table/model_accuracy

roslaunch deformable_manipulation_experiment_params generic_experiment_model_tests.launch task_type:=rope_table_linear_plus_x               disable_all_visualizations:=true start_bullet_viewer:=false --screen > ~/Dropbox/catkin_ws/src/smmap/logs/rope_table_linear_plus_x/model_accuracy/output_log.txt
roslaunch deformable_manipulation_experiment_params generic_experiment_model_tests.launch task_type:=rope_table_linear_minus_x              disable_all_visualizations:=true start_bullet_viewer:=false --screen > ~/Dropbox/catkin_ws/src/smmap/logs/rope_table_linear_minus_x/model_accuracy/output_log.txt
roslaunch deformable_manipulation_experiment_params generic_experiment_model_tests.launch task_type:=rope_table_linear_plus_y               disable_all_visualizations:=true start_bullet_viewer:=false --screen > ~/Dropbox/catkin_ws/src/smmap/logs/rope_table_linear_plus_y/model_accuracy/output_log.txt
roslaunch deformable_manipulation_experiment_params generic_experiment_model_tests.launch task_type:=rope_table_linear_minus_y              disable_all_visualizations:=true start_bullet_viewer:=false --screen > ~/Dropbox/catkin_ws/src/smmap/logs/rope_table_linear_minus_y/model_accuracy/output_log.txt
roslaunch deformable_manipulation_experiment_params generic_experiment_model_tests.launch task_type:=rope_table_linear_plus_z               disable_all_visualizations:=true start_bullet_viewer:=false --screen > ~/Dropbox/catkin_ws/src/smmap/logs/rope_table_linear_plus_z/model_accuracy/output_log.txt
roslaunch deformable_manipulation_experiment_params generic_experiment_model_tests.launch task_type:=rope_table_linear_pull_down_into_table disable_all_visualizations:=true start_bullet_viewer:=false --screen > ~/Dropbox/catkin_ws/src/smmap/logs/rope_table_linear_pull_down_into_table/model_accuracy/output_log.txt
