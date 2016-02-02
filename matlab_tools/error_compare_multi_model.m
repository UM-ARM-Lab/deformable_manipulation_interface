clc; clear;
%%
experiment = 'cloth_table';
base_dir = ['../logs/' experiment '/'];

%%
multi_model_error = load( [base_dir 'multi_model/error.txt'] );
time = load( [base_dir 'multi_model/time.txt'] );
time = time - time(1);

%%
deform_range = 10:0.5:20;
single_model_base_dir = [ base_dir 'deformability_range_10_to_20/' ];

% single_model_errors = zeros( length( multi_model_error ), length( deform_range ) );
single_model_errors = zeros( length(time), length(deform_range), length( deform_range ) );
for trans_deform_ind = 1:length( deform_range )
    for rot_deform_ind = 1:length( deform_range )
        single_model_errors( :, trans_deform_ind, rot_deform_ind ) = ...
            load( [single_model_base_dir sprintf( 'trans_%g_rot_%g/error.txt', deform_range( trans_deform_ind ), deform_range( rot_deform_ind ) )] );
    end
end
%%
% https://dgleich.github.io/hq-matlab-figs/
% http://blogs.mathworks.com/loren/2007/12/11/making-pretty-graphs/

width = 7;      % Width in inches
height = 6;     % Height in inches
alw = 0.7;      % AxesLineWidth
fsz = 18;       % Fontsize
lw = 1;         % LineWidth
msz = 12;       % MarkerSize

%%
close all;
h_single_model = [];
fig = figure( 'Units', 'inches', ...
              'Position', [0, 0, width, height] );
set( fig, 'PaperPositionMode', 'auto' );

h_multi_model = plot( time, multi_model_error );
hold on;

h_Xlabel = xlabel( 'Time (s)' );
h_Ylabel = ylabel( 'Error' );
% h_title = title( 'Colaborative folding results' );

for trans_deform_ind = length( deform_range ):-1:1
    for rot_deform_ind = length( deform_range ):-1:1
        h = plot( time, single_model_errors(:, trans_deform_ind, rot_deform_ind), ...
            'Color', [ 1, trans_deform_ind/(length( deform_range )+10), rot_deform_ind/(length( deform_range )+10)], ...
            'LineWidth', lw/4 );
        
        if rot_deform_ind == 1 && trans_deform_ind == 1
            h_single_model = h;
        end
        
        if deform_range(rot_deform_ind) == 14 && deform_range(trans_deform_ind) == 14
            h_manual_best = h;
            set( h, 'Color', [0, 1, 0], 'LineWidth', lw );
        end
    end
end

hold off

h_legend = legend( [h_multi_model, h_single_model, h_manual_best], ...
    'Multi Model', 'Single Model', ['Manual K = 14' repmat(char(3), 1, 2)] );

uistack( h_manual_best, 'top' );
uistack( h_multi_model, 'top' );

set([h_Xlabel, h_Ylabel, h_legend], ...
    'FontName'   , 'Helvetica');
set([gca, h_legend]            , ...
    'FontSize'   , fsz-2       );
set([h_Xlabel, h_Ylabel]       , ...
    'FontSize'   , fsz         );

output_name = ['output_images/' experiment '/multimodel-comparison.eps' ];
print( output_name, '-depsc2', '-r300');