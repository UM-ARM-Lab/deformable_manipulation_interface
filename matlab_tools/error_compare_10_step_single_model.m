clc; clear;
%%
experiment = 'colab_folding';
base_dir = ['../logs/' experiment '/'];

%%
experiment1.base_dir = [base_dir '1_step_trans_14_rot_14/'];
experiment1.error = load( [experiment1.base_dir 'error.txt'] );
experiment1.name = ['1 Step Jacobian' repmat(char(3), 1, 1)];
experiment1.time = load( [experiment1.base_dir 'time.txt'] );

experiment2.base_dir = [base_dir '10_step_trans_14_rot_14/'];
experiment2.error = load( [experiment2.base_dir 'error.txt'] );
experiment2.name = ['10 Step Jacobian' repmat(char(3), 1, 1)];
experiment2.time = load( [experiment2.base_dir 'time.txt'] );

t_start = min([experiment2.time; experiment1.time]);

experiment1.time = experiment1.time - t_start;
experiment2.time = experiment2.time - t_start;

output_name = ['output_images/' experiment '/1_step_vs_10_step' ];

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
h_legend = legend( experiment1.name, experiment2.name );

h_Xlabel = xlabel( 'Time (s)' );
h_Ylabel = ylabel( 'Error' );

set([h_Xlabel, h_Ylabel, h_legend], ...
    'FontName'   , 'Helvetica');
set([gca, h_legend]            , ...
    'FontSize'   , fsz-2       );
set([h_Xlabel, h_Ylabel]       , ...
    'FontSize'   , fsz         );

print( [output_name '.eps'], '-depsc2', '-r300');