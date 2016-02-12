clc; clear;
%%
experiment = 'colab_folding';
base_dir = ['../logs/' experiment '/'];

%%
experiment1.base_dir = [base_dir '1_step_single_model/trans_14_rot_14/'];
experiment1.error = load( [experiment1.base_dir 'error.txt'] );
experiment1.name = ['1 Step Pure Jacobian' repmat(char(3), 1, 1)];
experiment1.time = load( [experiment1.base_dir 'time.txt'] );

experiment2.base_dir = [base_dir 'shooting_max_1000_steps/'];
experiment2.error = load( [experiment2.base_dir 'error.txt'] );
experiment2.name = ['1 Step Shooting Method ' repmat(char(3), 1, 1)];
experiment2.time = load( [experiment2.base_dir 'time.txt'] );

experiment3.base_dir = [base_dir '1_step_multi_model/'];
experiment3.error = load( [experiment3.base_dir 'error.txt'] );
experiment3.name = ['1 Step Multi-model' repmat(char(3), 1, 1)];
experiment3.time = load( [experiment3.base_dir 'time.txt'] );

experiment4.base_dir = [base_dir '10_step_single_model/trans_14_rot_14/'];
experiment4.error = load( [experiment4.base_dir 'error.txt'] );
experiment4.name = ['10 Step Pure Jacobian' repmat(char(3), 1, 1)];
experiment4.time = load( [experiment4.base_dir 'time.txt'] );

experiment5.base_dir = [base_dir '10_step_multi_model/'];
experiment5.error = load( [experiment5.base_dir 'error.txt'] );
experiment5.name = ['10 Step Multi-model' repmat(char(3), 1, 1)];
experiment5.time = load( [experiment5.base_dir 'time.txt'] );

t_start = min([experiment1.time;
               experiment2.time;
               experiment3.time;
               experiment4.time;
               experiment5.time]);

experiment1.time = experiment1.time - t_start;
experiment2.time = experiment2.time - t_start;
experiment3.time = experiment3.time - t_start;
experiment4.time = experiment4.time - t_start;
experiment5.time = experiment5.time - t_start;

output_name = ['output_images/' experiment '/multiple-compare' ];

%%
% https://dgleich.github.io/hq-matlab-figs/
% http://blogs.mathworks.com/loren/2007/12/11/making-pretty-graphs/

width = 7;      % Width in inches
height = 6;     % Height in inches
alw = 0.7;      % AxesLineWidth
fsz = 18;       % Fontsize
lw = 2;         % LineWidth
msz = 12;       % MarkerSize

%%
close all;
fig = figure( 'Units', 'inches', ...
              'Position', [0, 0, width, height] );
set( fig, 'PaperPositionMode', 'auto' );

plot( experiment1.time, experiment1.error, 'b', 'LineWidth', lw )
hold on;
plot( experiment2.time, experiment2.error, 'g', 'LineWidth', lw )
plot( experiment3.time, experiment3.error, 'r', 'LineWidth', lw )
plot( experiment4.time, experiment4.error, 'c', 'LineWidth', lw )
plot( experiment5.time, experiment5.error, 'm', 'LineWidth', lw )
h_legend = legend( experiment1.name, ...
                   experiment2.name, ...
                   experiment3.name, ...
                   experiment4.name, ...
                   experiment5.name);

h_Xlabel = xlabel( 'Time (s)' );
h_Ylabel = ylabel( 'Error' );

set([h_Xlabel, h_Ylabel, h_legend], ...
    'FontName'   , 'Helvetica');
set([gca, h_legend]            , ...
    'FontSize'   , fsz-2       );
set([h_Xlabel, h_Ylabel]       , ...
    'FontSize'   , fsz         );

print( [output_name '.eps'], '-depsc2', '-r300');