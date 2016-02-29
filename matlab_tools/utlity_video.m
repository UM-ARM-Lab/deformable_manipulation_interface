clc; clear;
%%
experiment = 'rope_cylinder';
base_dir = ['../logs/' experiment '/'];

experiment1.base_dir = [ base_dir '1_step_single_model/' ];
experiment1.time = load( [experiment1.base_dir 'trans_14_rot_14/time.txt'] );
experiment1.name = ['Weighted Pseudoinverse' repmat(char(3), 1, 2)];

experiment1.time = experiment1.time - experiment1.time(1);

%%
log_folders = dir( experiment1.base_dir );

utility_image = [];
trans_list = [];
rot_list = [];

for folder_ind = 1:length(log_folders)

    parse_details = strsplit( log_folders(folder_ind).name, '_' );
    
    if ( length(parse_details) == 4 )
        trans_deform = str2double( parse_details{2} );
        rot_deform   = str2double( parse_details{4} );
        
        utility = load( [experiment1.base_dir '/' log_folders(folder_ind).name '/utility.txt'] );
        
%         x = trans_deform*2 - 19;
%         y = rot_deform*2 - 19;
        x = trans_deform*2 - 9;
        y = rot_deform*2 - 9;
        
        utility_image(x, y, :) = utility;
        trans_list = [trans_list trans_deform];
        rot_list = [rot_list rot_deform];
    end
    
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
trans_min = min( trans_list );
trans_max = max( trans_list );
rot_min = min( rot_list );
rot_max = max( rot_list );
log_utility_image = log( utility_image );

for t_ind = 1:length(experiment1.time)
    figure(1); clf;

    imagesc( linspace( rot_min, rot_max, size( utility_image, 2 ) ), ...
             linspace( trans_min, trans_max, size( utility_image, 1 ) ), ...
             -log_utility_image(:,:,t_ind) );

    set(gca, 'FontSize', fsz ); %<- Set properties
    set(gca,'YDir','normal')
    set(gca,'XDir','normal')

    axis square;

    colorbar;

    xlabel( 'Rotational Deformability' );
    ylabel( 'Translational Deformability' );
    title( sprintf( 'Negative Log Utility after %.2f seconds', experiment1.time(t_ind) ) );

    %%
    output_name = ['output_images/' experiment '/weighted_pseudoinverse_utility_over_time_video/' sprintf( 'time_%05.2f.eps', experiment1.time(t_ind) )];
    print( output_name, '-depsc2', '-r300');
end