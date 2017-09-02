#!/usr/bin/python

from mengyao_run_trial import *

def frange(start, stop, step):
    i = start
    while i < stop:
        yield i
        i += step

def mengyao_run_trials(experiment, generate_screenshots="false", log_prefix=""):

        # Note that this is 0 to 25 as range does [start, stop), thus we get 0:4:24 in Matlab speak
        # deform_range = range(0, 25, 4)
        # 5:5:25
        stretching_threshold_range = frange(0.35, 0.76, 0.5)
        rotation_deformability_range = frange(5, 26, 5)

        # Run the single model baseline
        for stretching_threshold in stretching_threshold_range:
            for rotation_deformability in rotation_deformability_range:
                mengyao_run_trial(experiment=experiment,
                          logging_enabled="true",
                          test_id=log_prefix + "MM_test/" + "stretching_" + str(stretching_threshold) + "_rot_" + str(rotation_deformability),
                          rotational_dis_deformability=rotation_deformability,
                          stretching_cosine_threshold=stretching_threshold)

        # Note that this is 0 to 16 as range does [start, stop), thus we get 0:1:10 in Matlab speak

if __name__ == '__main__':
    mengyao_run_trials("cloth_wafr")        
