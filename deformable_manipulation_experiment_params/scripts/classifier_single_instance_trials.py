#!/usr/bin/env python


import os
import sys
import subprocess
import numpy as np
import rospkg


def run_single_trial(experiment,
                     classifier_type,
                     classifier_dimension,
                     classifier_slice_type,
                     classifier_normalize_lengths,
                     classifier_normalize_connected_components,
                     bandits_logging_enabled=None,
                     controller_logging_enabled=None,
                     test_id=None,
                     task_max_time=None,
                     rrt_num_trials=None,
                     test_paths_in_bullet=None,
                     use_random_seed=None,
                     static_seed=None):
    # Constant values that we need
    roslaunch_args = ["roslaunch -p 16000 deformable_manipulation_experiment_params generic_experiment.launch",
                      "task_type:=" + experiment,
                      "classifier_type:=" + classifier_type,
                      "classifier_dimension:=" + classifier_dimension,
                      "classifier_slice_type:=" + classifier_slice_type,
                      "classifier_normalize_lengths:=" + classifier_normalize_lengths,
                      "classifier_normalize_connected_components:=" + classifier_normalize_connected_components,
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
               dim,
               slice,
               normalize_lengths,
               normalize_connected_components,
               run_knn=True,
               run_svm=True,
               run_dnn=True,
               run_voxnet=True,
               run_none=True,
               log_prefix="",
               rrt_num_trials=1):

    classifiers = []
    if run_knn:
        classifiers.append("kNN")
    if run_svm:
        classifiers.append("svm")
    if run_dnn:
        classifiers.append("dnn")
    if run_voxnet:
        classifiers.append("voxnet")
    if run_none:
        classifiers.append("none")

    if log_prefix != "":
        log_prefix += "/"

    foldername = slice + "__"
    if normalize_lengths:
        foldername += "normalized_lengths__"
    else:
        foldername += "raw_lengths__"
    if normalize_connected_components:
        assert (dim == 7)
        foldername += "normalized_connected_components"
    else:
        assert (dim == 13)
        foldername += "raw_connected_components"

    for classifier in classifiers:
        test_id = log_prefix + foldername + "/" + classifier
        run_single_trial(experiment=experiment,
                         classifier_type=classifier,
                         classifier_dimension=dim,
                         classifier_slice_type=slice,
                         classifier_normalize_lengths=str(normalize_lengths),
                         classifier_normalize_connected_components=str(normalize_connected_components),
                         test_id=test_id,
                         rrt_num_trials=str(rrt_num_trials),
                         test_paths_in_bullet="true",
                         use_random_seed="false")


if __name__ == "__main__":
    #num_trials = 10
    #np.random.seed(0xa8710913)
    #seeds = np.random.randint(low=0x1000000000000000, high=0x7fffffffffffffff, size=num_trials)
    seeds = None

    experiments = [
        "rope_hooks_simple",
        "rope_hooks",
        "engine_assembly",
        "rope_hooks_multi",
        "rope_hooks_simple_long_rope",
        "rope_hooks_simple_super_long_rope",
        "rope_hooks_simple_short_rope",
        # "cloth_hooks_simple",
        # "cloth_hooks_complex",
    ]

    slices = [
        "basic",
        # "extend_downwards_gravity_aligned",
        # "in_plane_gravity_aligned",
    ]

    for slice in slices:
        for experiment in experiments:
            run_trials(experiment=experiment,
                       dim=13,
                       slice=slice,
                       normalize_lengths=True,
                       normalize_connected_components=False,
                       log_prefix="icra_planning_time_trials",
                       rrt_num_trials=100,
                       run_knn=True,
                       run_svm=True,
                       run_dnn=True,
                       run_voxnet=True,
                       run_none=True)
