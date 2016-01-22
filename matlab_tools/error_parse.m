clc; clear;
%%
% https://dgleich.github.io/hq-matlab-figs/


% Defaults for this blog post
width = 3;     % Width in inches
height = 2.3;    % Height in inches
alw = 1;       % AxesLineWidth
fsz = 20;      % Fontsize
lw = 2;        % LineWidth
msz = 12;      % MarkerSize

% % The properties we've been using in the figures
% set(0,'defaultLineLineWidth',lw);   % set the default line width to lw
% set(0,'defaultLineMarkerSize',msz); % set the default line marker size to msz
% set(0,'defaultLineLineWidth',lw);   % set the default line width to lw
% set(0,'defaultLineMarkerSize',msz); % set the default line marker size to msz
% 
% % Set the default Size for display
% defpos = get(0,'defaultFigurePosition');
% set(0,'defaultFigurePosition', [defpos(1) defpos(2) width*100, height*100]);
% 
% % Set the defaults for saving/printing to a file
% set(0,'defaultFigureInvertHardcopy','on'); % This is the default anyway
% set(0,'defaultFigurePaperUnits','inches'); % This is the default anyway
% defsize = get(gcf, 'PaperSize');
% left = (defsize(1)- width)/2;
% bottom = (defsize(2)- height)/2;
% defsize = [left, bottom, width, height];
% set(0, 'defaultFigurePaperPosition', defsize);


%%
experiment = 'rope_cylinder';
log_folders = dir( experiment );

error_image = [];
trans_list = [];
rot_list = [];

for folder_ind = 1:length(log_folders)

    parse_details = strsplit( log_folders(folder_ind).name, '_' );
    
    if ( length(parse_details) == 4 )
        trans_deform = str2double( parse_details{2} );
        rot_deform   = str2double( parse_details{4} );
        
        errors = load( [experiment '/' log_folders(folder_ind).name '/error.txt'] );
        
        x = (trans_deform * 2) - 9;
        y = (rot_deform * 2) - 9;
        
        error_image(x, y) = errors( floor( length(errors)/2 ) );
        trans_list = [trans_list trans_deform];
        rot_list = [rot_list rot_deform];
    end
    
end
%%
figure(1); clf;

pos = get(gcf, 'Position');
set(gcf, 'Position', [pos(1) pos(2) width*100, height*100]); %<- Set size
set(gca, 'FontSize', fsz, 'LineWidth', alw); %<- Set properties

time = load( [experiment '/' log_folders(folder_ind).name '/time.txt'] );
mid_time_val = time( floor( length(errors)/2 ) ) - time(1);

trans_min = min( trans_list );
trans_max = max( trans_list );
rot_min = min( rot_list );
rot_max = max( rot_list );

imagesc( linspace( rot_min, rot_max, size( error_image, 2 ) ), ...
         linspace( trans_min, trans_max, size( error_image, 1 ) ), ...
         error_image );
axis square;

set(gcf,'InvertHardcopy','on');
set(gcf,'PaperUnits', 'inches');
papersize = get(gcf, 'PaperSize');
left = (papersize(1)- width)/2;
bottom = (papersize(2)- height)/2;
myfiguresize = [left, bottom, width, height];
set(gcf,'PaperPosition', myfiguresize);

colorbar;
set(gca,'YDir','normal')
set(gca,'XDir','normal')
xlabel( 'Rotational Deformability' );
ylabel( 'Translational Deformability' );
title( sprintf( 'Error after %.2f seconds', mid_time_val ) );

print( experiment, '-dpng', '-r300');