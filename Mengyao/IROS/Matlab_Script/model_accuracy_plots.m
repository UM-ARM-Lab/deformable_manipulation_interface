clear all

space_hold_1 = '%n';
space_hold_2 = '%n %n';

% basename = '/home/dmcconac/Dropbox/catkin_ws/src/smmap/logs/';
basename = 'E:/Dropbox/catkin_ws/src/smmap/logs/';
folder = 'rope_table_IROS_model_accuracy_test/model_accuracy/';

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
plot(time, constrained_model_error,...
     time, diminishing_rigidity_model_error, 'LineWidth', 2)
legend(parameter_set_mm, parameter_set_dd_wo, 'Location', 'northeast')

xlabel('Time (s)')
ylabel('Model Prediction Error')
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
    
    
    plot(-1, -1, -1, -1, 'LineWidth', 2)
    axis([0   10.0000         0    0.0150]);
    legend(parameter_set_mm, parameter_set_dd_wo, 'Location', 'northeast')
    xlabel('Time (s)')
    ylabel('Model Prediction Error')
    
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
        
        axis([0   10.0000         0    0.0150]);
        legend(parameter_set_mm, parameter_set_dd_wo, 'Location', 'northeast')
        xlabel('Time (s)')
        ylabel('Model Prediction Error')
        drawnow;
        
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