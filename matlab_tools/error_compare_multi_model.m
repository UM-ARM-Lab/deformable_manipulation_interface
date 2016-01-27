clc; clear;
%%
experiment = 'colab_folding';
base_dir = ['../logs/' experiment '/'];

%%
% multi_model_error = load( [base_dir 'multi_model/error.txt'] );
% time = load( [base_dir 'multi_model/time.txt'] );
% time = time - time(1);

%%
deform_range = 10:0.5:20;
single_model_base_dir = [ base_dir 'deformability_range_10_to_20/' ];

% single_model_errors = zeros( length( multi_model_error ), length( deform_range ) );
single_model_errors = zeros( 399, length( deform_range ) );
for deform_ind = 1:length( deform_range )
     single_model_errors( :, deform_ind ) = load( [single_model_base_dir sprintf( 'trans_%g_rot_%g/error.txt', deform_range( deform_ind ), deform_range( deform_ind ) )] );
end
%%
% https://dgleich.github.io/hq-matlab-figs/

width = 7;      % Width in inches
height = 6;     % Height in inches
alw = 1;        % AxesLineWidth
fsz = 24;       % Fontsize
lw = 2;         % LineWidth
msz = 12;       % MarkerSize

% The properties we've been using in the figures
set(0,'defaultLineLineWidth',lw);   % set the default line width to lw
set(0,'defaultLineMarkerSize',msz); % set the default line marker size to msz
set(0,'defaultLineLineWidth',lw);   % set the default line width to lw
set(0,'defaultLineMarkerSize',msz); % set the default line marker size to msz

% Set the default Size for display
defpos = get(0,'defaultFigurePosition');
set(0,'defaultFigurePosition', [defpos(1) defpos(2) width*100, height*100]);

% Set the defaults for saving/printing to a file
set(0,'defaultFigureInvertHardcopy','on'); % This is the default anyway
set(0,'defaultFigurePaperUnits','inches'); % This is the default anyway
defsize = get(gcf, 'PaperSize');
left = (defsize(1)- width)/2;
bottom = (defsize(2)- height)/2;
defsize = [left, bottom, width, height];
set(0, 'defaultFigurePaperPosition', defsize);

%%
figure(1); clf;

% plot( time, multi_model_error );
hold on

set(gca, 'FontSize', fsz ); %<- Set properties
xlabel( 'Time (s)' );
ylabel( 'Error' );
% title( 'Colaborative folding results' );

for deform_ind = 1:length( deform_range )
    plot( single_model_errors(:,deform_ind), 'Color', [ 1, deform_ind/(length( deform_range )+10), 0] );
end

legend( 'Multi model (follows ''best'' single model)', 'Single model (darker is more rigid)' );

hold off

%%
output_name = ['output_images/' experiment '/multimodel-comparison.eps' ];
print( output_name, '-depsc2', '-r300');