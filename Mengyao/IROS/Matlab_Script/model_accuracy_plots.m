clear all

space_hold_1 = '%n';
space_hold_2 = '%n %n';

% basename = '/home/dmcconac/Dropbox/catkin_ws/src/smmap/logs/';
basename = 'E:/Dropbox/catkin_ws/src/smmap/logs/';
folders = {
    'rope_table_linear_plus_x/model_accuracy',
    'rope_table_linear_minus_x/model_accuracy',
%     'rope_table_linear_plus_y/model_accuracy',
%     'rope_table_linear_minus_y/model_accuracy',
%     'rope_table_linear_plus_z/model_accuracy',
%     'rope_table_linear_pull_down_into_table/model_accuracy'
    };

titles = {
    'Pushing towards rope',
    'Pulling away from rope'
    };
%%
%%%%%%%%%%% Error Analysis %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

parameter_set_mm = 'New';
parameter_set_dd_wo = 'Baseline';

for ind = 1:length(folders)
    folder = folders{ind};
    error_filename = [basename, folder, '/model_prediction_error_unweighted.txt']
    time_filename = [basename, folder, '/control_time.txt'];
    image_path = [basename, folder, '/model_accuracies.png'];
    
    [constrained_model_error, diminishing_rigidity_model_error] = ...
        textread(error_filename, space_hold_2, 'headerlines', 1);
    time = textread(time_filename, space_hold_1, 'headerlines', 1);
    
    fig = figure(ind);
    plot(time, sqrt(constrained_model_error),...
         time, sqrt(diminishing_rigidity_model_error),'LineWidth',2)
    legend(parameter_set_mm, parameter_set_dd_wo, 'Location', 'northeast')
%     title(titles{ind})
    xlabel('Time (s)')
    ylabel('Model Prediction Error')
    saveas(fig, image_path)
end