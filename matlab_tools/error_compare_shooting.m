clc; clear;
%%
base_dir = '../logs/';
experiment1.log_folder = [base_dir 'colab_folding_small_range_of_deformability/trans_14_rot_14'];
experiment1.name = 'Pure Jacobian based';
experiment1.error = load( [experiment1.log_folder '/error.txt'] );

experiment2.log_folder = [base_dir 'colab_folding_shooting_10_ittr_trans_14_rot_14'];
experiment2.name = '10 step Gradient Update';
experiment2.error = load( [experiment2.log_folder '/error.txt'] );

time = load( [experiment1.log_folder '/time.txt'] );
time = time - time(1);

%%
plot( time, [ experiment1.error, experiment2.error ] )