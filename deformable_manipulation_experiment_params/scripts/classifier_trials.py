#!/usr/bin/env python3


import os
import sys
import subprocess
import numpy as np


def ensure_dir(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)


def run_single_trial(experiment,
                     classifier_type,
                     bandits_logging_enabled=None,
                     controller_logging_enabled=None,
                     test_id=None,
                     task_max_time=None,
                     rrt_num_trials=None,
                     use_random_seed=None,
                     static_seed=None):
    # Constant values that we need
    roslaunch_args = ["roslaunch deformable_manipulation_experiment_params generic_experiment.launch",
                      "task_type:=" + experiment,
                      "classifier_type:=" + classifier_type,
                      "launch_simulator:=true",
                      "start_bullet_viewer:=false",
                      "screenshots_enabled:=false",
                      "launch_planner:=true",
                      "disable_smmap_visualizations:=true"]

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

    # Add any extra parameters that have been added
    roslaunch_args += sys.argv[1:]

    log_folder = "/home/dmcconac/Dropbox/catkin_ws/src/smmap/logs/" + experiment + "/" + test_id
    ensure_dir(log_folder)
    roslaunch_args.append("    --screen")
    cmd = " ".join(roslaunch_args)
    print(cmd, "\n")

    # output = subprocess.run(args=roslaunch_args, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    with open(log_folder + "/smmap_output.log", "wb") as file:
        process = subprocess.Popen(args=cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        while True:
            line = process.stdout.readline()
            if not line:
                break
            sys.stdout.write(line.decode("utf-8"))
            file.write(line)


def run_trials(experiment,
               run_none=True,
               run_knn=True,
               run_svm=True,
               run_dnn=True,
               seeds=None,
               log_prefix="",
               rrt_num_trials=1):

    classifiers = []

    if run_knn:
        classifiers.append("kNN")

    if run_svm:
        classifiers.append("svm")

    if run_dnn:
        classifiers.append("dnn")

    # Run "none" last in case it takes forever to succeed 10 times
    if run_none:
        classifiers.append("none")

    if log_prefix != "":
        log_prefix += "/"

    for classifier in classifiers:
        if seeds is not None:
            for seed in seeds:
                run_single_trial(experiment=experiment,
                                 classifier_type=classifier,
                                 test_id=log_prefix + classifier + "_" + hex(seed),
                                 rrt_num_trials=str(rrt_num_trials),
                                 use_random_seed="false",
                                 static_seed=hex(seed))
        else:
            run_single_trial(experiment=experiment,
                             classifier_type=classifier,
                             test_id=log_prefix + "/" + classifier,
                             rrt_num_trials=rrt_num_trials,
                             use_random_seed="false")


if __name__ == "__main__":
    num_trials = 10
    np.random.seed(0xa8710913)
    seeds = np.random.randint(low=0x1000000000000000, high=0x7fffffffffffffff, size=num_trials)
    run_trials(experiment="rope_hooks", seeds=seeds, log_prefix="full_task_trials")