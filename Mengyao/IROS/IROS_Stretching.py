#!/usr/bin/python

from mengyao_run_trial import *

from numpy import arange as frange


def mengyao_run_trials(experiment, generate_screenshots="false", log_prefix=""):

        down_scale = 400
        stretching_threshold = 0.4
        
        trans_dir_deformability = 900 
        rotation_deformability = 20
        trans_dis_deformability = 10


        ####### TODO: Another Variable(s) should be defined (in mengyao_run_trial.py) to set the type of model controller 
        ####### for each of the functions below:
        ####### For each function below, only one of the bench mark or new model should be run. 
        ####### The resulting "realtime_stretching_factor.txt" will have only one column recording the data for 
        ####### the one controller.

        ####### TODO: Put the Matlab scripts in the Matlab_Script folder into the corresponding folder
        ####### e.g: smmap/log/cloth_single_pole/control_error_IROS_cloth_single_pole.m
        #######      smmap/log/cloth_single_pole/single_stretching_factor_IROS_cloth_single_pole.m

        ####### TODO: Double check the log setting here. 

        # rope_cylinder on BenchMark model
        mengyao_run_trial(experiment = "rope_cylinder_two_grippers",
          start_bullet_viewer = "false",
          controller_logging_enabled = "true",
          test_id = log_prefix + ("stretching_status/BM"),
          desired_down_scale = down_scale,
          translational_dir_deformability = trans_dir_deformability,
          translational_dis_deformability = trans_dis_deformability,
          rotational_dis_deformability = rotation_deformability,
          stretching_cosine_threshold = stretching_threshold,
          )

        # rope_cylinder on new model
        mengyao_run_trial(experiment = "rope_cylinder_two_grippers",
          start_bullet_viewer = "false",
          controller_logging_enabled = "true",
          test_id = log_prefix + ("stretching_status/NM"),
          desired_down_scale = down_scale,
          translational_dir_deformability = trans_dir_deformability,
          translational_dis_deformability = trans_dis_deformability,
          rotational_dis_deformability = rotation_deformability,
          stretching_cosine_threshold = stretching_threshold,
          )

        down_scale = 400        
        # cloth_single_pole on Bench mark model
        mengyao_run_trial(experiment = "cloth_single_pole",
          start_bullet_viewer = "false",
          controller_logging_enabled = "true",
          test_id = log_prefix + ("stretching_status/BM"),
          desired_down_scale = down_scale,
          translational_dir_deformability = trans_dir_deformability,
          translational_dis_deformability = trans_dis_deformability,
          rotational_dis_deformability = rotation_deformability,
          stretching_cosine_threshold = stretching_threshold,
          )

        # cloth_single_pole on new model with cos = 0.4 (stretching parameter)
        mengyao_run_trial(experiment = "cloth_single_pole",
          start_bullet_viewer = "false",
          controller_logging_enabled = "true",
          test_id = log_prefix + ("stretching_status/NM/cos_04"),
          desired_down_scale = down_scale,
          translational_dir_deformability = trans_dir_deformability,
          translational_dis_deformability = trans_dis_deformability,
          rotational_dis_deformability = rotation_deformability,
          stretching_cosine_threshold = stretching_threshold,
          )

        # cloth_single_pole on new model with cos = 0.6 (stretching parameter)
        stretching_threshold = 0.6
        mengyao_run_trial(experiment = "cloth_single_pole",
          start_bullet_viewer = "false",
          controller_logging_enabled = "true",
          test_id = log_prefix + ("stretching_status/NM/cos_06"),
          desired_down_scale = down_scale,
          translational_dir_deformability = trans_dir_deformability,
          translational_dis_deformability = trans_dis_deformability,
          rotational_dis_deformability = rotation_deformability,
          stretching_cosine_threshold = stretching_threshold,
          )

        # cloth_single_pole on new model with cos = 0.8 (stretching parameter)
        stretching_threshold = 0.8
        mengyao_run_trial(experiment = "cloth_single_pole",
          start_bullet_viewer = "false",
          controller_logging_enabled = "true",
          test_id = log_prefix + ("stretching_status/NM/cos_08"),
          desired_down_scale = down_scale,
          translational_dir_deformability = trans_dir_deformability,
          translational_dis_deformability = trans_dis_deformability,
          rotational_dis_deformability = rotation_deformability,
          stretching_cosine_threshold = stretching_threshold,
          )




if __name__ == '__main__':
  mengyao_run_trials("***")        
