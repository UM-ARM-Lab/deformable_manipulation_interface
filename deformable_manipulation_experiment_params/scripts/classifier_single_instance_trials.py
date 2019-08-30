#!/usr/bin/env python


import os
import sys
import subprocess
import numpy as np
import rospkg


def run_single_trial(experiment,
                     classifier_type,
                     classifier_dim,
                     classifier_slice_type,
                     bandits_logging_enabled=None,
                     controller_logging_enabled=None,
                     test_id=None,
                     task_max_time=None,
                     rrt_num_trials=None,
                     test_paths_in_bullet=None,
                     use_random_seed=None,
                     static_seed=None):
    # Constant values that we need
    roslaunch_args = ["roslaunch deformable_manipulation_experiment_params generic_experiment.launch",
                      "task_type:=" + experiment,
                      "classifier_type:=" + classifier_type,
                      "classifier_dim:=" + classifier_dim,
                      "classifier_slice_type:=" + classifier_slice_type,
                      "launch_simulator:=true",
                      "start_bullet_viewer:=false",
                      "screenshots_enabled:=false",
                      "launch_planner:=true",
                      "disable_smmap_visualizations:=true",
                      "rerun_forever:=false"]

    # Setup logging parameters
    if bandits_logging_enabled is not None:
        roslaunch_args.append("bandits_logging_enabled:=" + bandits_logging_enabled)
    if controller_logging_enabled is not None:
        roslaunch_args.append("bandits_logging_enabled:=" + controller_logging_enabled)
    if test_id is not None:
        roslaunch_args.append("test_id:=" + test_id)

    # Task settings
    if task_max_time is not None:
        roslaunch_args.append("task_max_time_override:=true")
        roslaunch_args.append("task_max_time:=" + task_max_time)

    # Randomization parameters
    if use_random_seed is not None:
        roslaunch_args.append("use_random_seed:=" + use_random_seed)
    if static_seed is not None:
        assert(use_random_seed is None or
               use_random_seed == False or
               use_random_seed == "false" or
               use_random_seed == "False")
        roslaunch_args.append("use_random_seed:=false")
        roslaunch_args.append("static_seed:=" + static_seed)

    if rrt_num_trials is not None:
        roslaunch_args.append("rrt_num_trials:=" + rrt_num_trials)

    if test_paths_in_bullet is not None:
        roslaunch_args.append("test_paths_in_bullet:=" + test_paths_in_bullet)

    # Add any extra parameters that have been added
    roslaunch_args += sys.argv[1:]

    # Determine save locations
    smmap_folder = rospkg.RosPack().get_path("smmap")

    log_folder = smmap_folder + "/logs/" + experiment + "/" + test_id
    subprocess.call(args="mkdir -p " + log_folder, shell=True)
    roslaunch_args.append("    --screen")
    cmd = " ".join(roslaunch_args)
    print(cmd)
    print("Logging to " + log_folder + "/smmap_output.log\n")

    with open(log_folder + "/smmap_output.log", "wb") as file:
        process = subprocess.Popen(args=cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        while True:
            line = process.stdout.readline()
            if not line:
                break
            file.write(line)


def run_trials(experiment,
               dim_slice=None,
               run_knn=True,
               run_svm=True,
               run_dnn=True,
               run_none=True,
               seeds=None,
               log_prefix="",
               rrt_num_trials=1):

    if dim_slice == None or len(dim_slice) == 0:
        dim_slice =[("13", "basic")]

    classifiers = []
    if run_knn:
        classifiers.append("kNN")
    if run_svm:
        classifiers.append("svm")
    if run_dnn:
        classifiers.append("dnn")
    if run_none:
        classifiers.append("none")

    if log_prefix != "":
        log_prefix += "/"

    for dim, slice in dim_slice:
        for classifier in classifiers:
            if seeds is not None:
                for seed in seeds:
                    run_single_trial(experiment=experiment,
                                     classifier_type=classifier,
                                     classifier_dim=dim,
                                     classifier_slice_type=slice,
                                     test_id=log_prefix + dim + "feature__" + slice + "__" + classifier + "_" + hex(seed)[:-1],
                                     rrt_num_trials=str(rrt_num_trials),
                                     use_random_seed="false",
                                     static_seed=hex(seed)[:-1])
            else:
                run_single_trial(experiment=experiment,
                                 classifier_type=classifier,
                                 classifier_dim=dim,
                                 classifier_slice_type=slice,
                                 test_id=log_prefix + dim + "feature__" + slice + "__" + classifier,
                                 rrt_num_trials=str(rrt_num_trials),
                                 test_paths_in_bullet="true",
                                 use_random_seed="false")


if __name__ == "__main__":
    #num_trials = 10
    #np.random.seed(0xa8710913)
    #seeds = np.random.randint(low=0x1000000000000000, high=0x7fffffffffffffff, size=num_trials)

    seeds = None
    dim_slice = [
        ("13", "basic"),
        ("7", "basic"),
        # ("7", "in_plane_gravity_aligned"),
        # ("7", "in_plane_gripper_aligned"),
        # ("7", "extend_downwards_gravity_aligned"),
        # ("7", "extend_downwards_gripper_aligned"),
    ]
    run_trials(experiment="rope_hooks_simple",
               dim_slice=dim_slice,
               seeds=seeds,
               log_prefix="end_to_end_planning_time_script_test",
               rrt_num_trials=100)
