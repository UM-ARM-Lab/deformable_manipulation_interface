#!/usr/bin/env python


import os
import sys
import subprocess
import numpy as np
import rospkg


def run_single_trial(experiment,
                     classifier_type,
                     bandits_logging_enabled=None,
                     controller_logging_enabled=None,
                     test_id=None,
                     task_max_time=None,
                     generate_test_data=None,
                     generate_trajectories=None,
                     visualize_incomplete_trajectories=None,
                     generate_transitions=None,
                     generate_meaningful_mistakes=None,
                     generate_features=None,
                     test_classifier=None,
                     rrt_num_trials=None,
                     use_random_seed=None,
                     static_seed=None):
    # Constant values that we need
    roslaunch_args = ["roslaunch deformable_manipulation_experiment_params transition_learning.launch",
                      "task_type:=" + experiment,
                      "classifier_type:=" + classifier_type,
                      "launch_simulator:=true",
                      "start_bullet_viewer:=false",
                      "screenshots_enabled:=false",
                      "launch_tester:=true",
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

    if generate_test_data is not None:
        roslaunch_args.append("generate_test_data:=" + generate_test_data)
    if generate_trajectories is not None:
        roslaunch_args.append("generate_trajectories:=" + generate_trajectories)
    if visualize_incomplete_trajectories is not None:
        roslaunch_args.append("visualize_incomplete_trajectories:=" + visualize_incomplete_trajectories)
    if generate_transitions is not None:
        roslaunch_args.append("generate_transitions:=" + generate_transitions)
    if generate_meaningful_mistakes is not None:
        roslaunch_args.append("generate_meaningful_mistakes:=" + generate_meaningful_mistakes)
    if generate_features is not None:
        roslaunch_args.append("generate_features:=" + generate_features)

    if test_classifier is not None:
        roslaunch_args.append("test_classifier:=" + test_classifier)
    if rrt_num_trials is not None:
        roslaunch_args.append("rrt_num_trials:=" + rrt_num_trials)

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
               run_none=True,
               run_knn=True,
               run_svm=True,
               run_dnn=True,
               seed=None,
               log_prefix="",
               rrt_num_trials=1):

    classifiers = []

    # Run 'none' first so that we can shortcut early if 'none' performs too well
    if run_none:
        classifiers.append("none")

    if run_knn:
        classifiers.append("kNN")

    if run_svm:
        classifiers.append("svm")

    if run_dnn:
        classifiers.append("dnn")

    if log_prefix != "":
        log_prefix += "/"

    for classifier in classifiers:
        run_single_trial(experiment=experiment,
                         classifier_type=classifier,
                         test_id=log_prefix + "/" + classifier,
                         test_classifier="true",
                         rrt_num_trials=str(rrt_num_trials),
                         use_random_seed="false",
                         static_seed=seed)


if __name__ == "__main__":
    #num_trials = 10
    #np.random.seed(0xa8710913)
    #seeds = np.random.randint(low=0x1000000000000000, high=0x7fffffffffffffff, size=num_trials)
    seeds = None
    run_trials(experiment="rope_hooks_multi", seeds=seeds, log_prefix="classifier_test_end_to_end__fewer_hooks", rrt_num_trials=100)
