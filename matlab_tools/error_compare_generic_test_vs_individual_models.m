clc; clear;
%%
experiment = 'cloth_table';
% experiment = 'colab_folding';
% experiment = 'rope_cylinder';

if experiment(1) == 'c'
    deform_range = (10:2:18)';
else
    deform_range = (6:2:14)';
end

base_dir = ['../logs/' experiment '/' ];

%%
experiment2.base_dir = [base_dir ''];
experiment2.error = load( [experiment2.base_dir 'error.txt'] );
experiment2.name = ['Generic Test' repmat(char(3), 1, 1)];
experiment2.time = load( [experiment2.base_dir 'time.txt'] );
% experiment2.model_chosen = load( [experiment2.base_dir 'utility.txt'] );
% experiment2.trans_chosen = deform_range( floor( experiment2.model_chosen / 5 ) + 1 );
% experiment2.rot_chosen = deform_range( mod( experiment2.model_chosen, 5 ) + 1 );

%%
experiment1.base_dir = [ base_dir '/1_step_kalman_trials/single_model/' ];
experiment1.time = load( [experiment1.base_dir 'trans_14_rot_14/time.txt'] );
experiment1.error = zeros( length(experiment1.time), length(deform_range), length( deform_range ) );
experiment1.name = ['Single-model' repmat(char(3), 1, 1)];

for trans_deform_ind = 1:length( deform_range )
    for rot_deform_ind = 1:length( deform_range )
        experiment1.error( :, trans_deform_ind, rot_deform_ind ) = ...
            load( [experiment1.base_dir sprintf( 'trans_%g_rot_%g/error.txt', ...
                                                 deform_range( trans_deform_ind ), ...
                                                 deform_range( rot_deform_ind ) )] );
    end
end

%%
t_start = min([experiment1.time;
               experiment2.time ]);

experiment1.time = experiment1.time - t_start;
experiment2.time = experiment2.time - t_start;

output_name = ['output_images/' experiment '/generic_compare' ];

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

hold on;

for trans_deform_ind = length( deform_range ):-1:1
    for rot_deform_ind = length( deform_range ):-1:1
        h = plot( experiment1.time, experiment1.error(:, trans_deform_ind, rot_deform_ind), ...
            'Color', [ 1, trans_deform_ind/(length( deform_range )+10), rot_deform_ind/(length( deform_range )+10)], ...
            'LineWidth', lw/4 );
        
        if rot_deform_ind == 1 && trans_deform_ind == 1
            experiment1.plot_handle = h;
        end
        
        if deform_range(rot_deform_ind) == mean( deform_range ) ...
                && deform_range(trans_deform_ind) == mean( deform_range )
            h_manual_best = h;
            set( h, 'Color', 'b', 'LineWidth', lw );
        end
    end
end

uistack( h_manual_best, 'top' );

experiment2.plot_handle = plot( experiment2.time, experiment2.error, 'g', 'LineWidth', lw );
hold off

h_legend = legend( [experiment1.plot_handle, experiment2.plot_handle, h_manual_best], ...
    experiment1.name, experiment2.name, ['Manual K = ' sprintf('%d', mean( deform_range )) repmat(char(3), 1, 1)] );

h_Xlabel = xlabel( 'Time (s)' );
h_Ylabel = ylabel( 'Error' );
% h_title = title( 'Colaborative folding results' );


set([h_Xlabel, h_Ylabel, h_legend], ...
    'FontName'   , 'Helvetica');
set([gca, h_legend]            , ...
    'FontSize'   , fsz-2       );
set([h_Xlabel, h_Ylabel]       , ...
    'FontSize'   , fsz         );

print( [output_name '.eps'], '-depsc2', '-r300');