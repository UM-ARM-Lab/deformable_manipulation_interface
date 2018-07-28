clear all

space_hold_1 = '%n';
space_hold_2 = '%n %n';

% basename = '/home/dmcconac/Dropbox/catkin_ws/src/smmap/logs/';
basename = 'E:/Dropbox/catkin_ws/src/smmap/logs/';
folder = 'cloth_table_IROS_model_accuracy_test/default/';

%%
%%%%%%%%%%% Error Analysis %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

parameter_set_mm = 'New';
parameter_set_dd_wo = 'Baseline';

error_filename = [basename, folder, 'model_prediction_error_unweighted.txt'];
time_filename = [basename, folder, 'control_time.txt'];
image_path = [basename, folder, 'model_accuracies.png'];

[constrained_model_error_squared, diminishing_rigidity_model_error_squared] = ...
    textread(error_filename, space_hold_2, 'headerlines', 1);
constrained_model_error = sqrt(constrained_model_error_squared);
diminishing_rigidity_model_error = sqrt(diminishing_rigidity_model_error_squared);
time = textread(time_filename, space_hold_1, 'headerlines', 1);
time = time - time(1);

fig = figure(1);
set(gcf,'PaperUnits', 'inches');
papersize = get(gcf, 'PaperSize');

plot(time, constrained_model_error,...
     time, diminishing_rigidity_model_error, 'LineWidth', 2)
legend(parameter_set_mm, parameter_set_dd_wo, 'Location', 'northwest')
axis([0   5.50000         0    0.2500]);

xlabel('Time (s)')
ylabel('Model Prediction Error (m)')
saveas(fig, image_path)

%%
%%%%%%%%%%%%% Video generation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
make_video = true;
video_path = [basename, folder, 'model_accuracies_video'];
pause_len = time(2) - time(1);

if make_video
    try
        votype = 'avifile';
        vo = avifile([video_path '.avi'], 'fps', 1/pause_len);
    catch
        votype = 'VideoWriter';
        vo = VideoWriter(video_path, 'MPEG-4');
        set(vo, 'FrameRate', 1/pause_len);
        open(vo);
    end
    
    
    h_new = plot(-1, -1, 'LineWidth', 2); hold on
    h_baseline = plot(-1, -1, 'LineWidth', 2); hold off
%     axis([0   10.0000         0    0.0150]);
    axis([0   5.50000         0    0.2500]);
    h_legend = legend(parameter_set_mm, parameter_set_dd_wo, 'Location', 'northwest');
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('Model Prediction Error');
    
    set([gca, h_xlabel, h_ylabel, h_legend], 'FontSize', 24);
    
    h_new.XDataSource = 'time_slice';
    h_new.YDataSource = 'new_error_slice';
    h_baseline.XDataSource = 'time_slice';
    h_baseline.YDataSource = 'baseline_error_slice';
    
    F = getframe(gcf);
    
    switch votype
        case 'avifile'
            vo = addframe(vo, F);
        case 'VideoWriter'
            writeVideo(vo, F);
        otherwise
            error('unrecognized votype');
    end
    
    
    
    
    for ind = 1:length(time)    
        plot(time(1:ind), constrained_model_error(1:ind), ...
             time(1:ind), diminishing_rigidity_model_error(1:ind), 'LineWidth', 2)
         
        axis([0   5.50000         0    0.2500]);
        h_legend = legend(parameter_set_mm, parameter_set_dd_wo, 'Location', 'northwest');
        h_xlabel = xlabel('Time (s)');
        h_ylabel = ylabel('Model Prediction Error');
    
        set([gca, h_xlabel, h_ylabel, h_legend], 'FontSize', 24);
         
%         time_slice = time(1:ind);
%         new_error_slice = constrained_model_error(1:ind);
%         baseline_error_slice = diminishing_rigidity_model_error(1:ind);

%         refreshdata;
        drawnow;
        pause(0.01)
        
        F = getframe(gcf);
        
        switch votype
            case 'avifile'
                vo = addframe(vo, F);
            case 'VideoWriter'
                writeVideo(vo, F);
            otherwise
                error('unrecognized votype');
        end
    end
    
    close(vo);
end