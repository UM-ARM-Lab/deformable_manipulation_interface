close all
clear all

space_hold_1 = '%n';
space_hold_2 = '%n %n';

%%%%%%%%%%% Error Analysis %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
file_error_1 = 'controll_error/control_error_realtime.txt';
file_t_1 = 'controll_error/cloth_single_pole/control_time.txt';

parameter_set_dd_wo = 'e^r_B';
parameter_set_mm = 'e^r_N';

[mean_error_mm, mean_error_dd_wo]  = readData_fn(file_error_1, space_hold_2);

[t_1] = textread(file_t_1, space_hold_1, 'headerlines',1);
t_dd_wo = t_1;

t_mm = t_1;
fig_path_integral_relative_error = 'controll_error/integral_relative_error_MM.png';

real_time_sum_mm = zeros(length(t_mm),1);
real_time_sum_dd_wo = real_time_sum_mm;

real_time_sum_mm(1) = mean_error_mm(1); 
real_time_sum_dd_wo(1) = mean_error_dd_wo(1);     

min_value = zeros(length(t_mm),1);
min_ind = min_value;
min_value(1) = mean_error_mm(1);

% Find the miminum error
for i = 2:length(t_mm)
   real_time_sum_mm(i) = real_time_sum_mm(i-1) + mean_error_mm(i); 
   real_time_sum_dd_wo(i) = real_time_sum_dd_wo(i-1) + mean_error_dd_wo(i);     

   if mean_error_mm(i) < mean_error_dd_wo(i)
       min_value(i) = mean_error_mm(i);
       min_ind(i) = 1;
   else
       min_value(i) = mean_error_dd_wo(i);
       min_ind(i) = 3;
   end
   
end

relative_error_mm = mean_error_mm - min_value;
relative_error_dd_wo = mean_error_dd_wo - min_value;

relative_sum_mm = zeros(length(t_mm),1);
relative_sum_dd_wo = zeros(length(t_mm),1);

relative_sum_mm(1) = relative_error_mm(1);
relative_sum_dd_wo(1) = relative_error_dd_wo(1);

for i = 2:length(t_mm)
    relative_sum_mm(i) = relative_sum_mm(i-1) + relative_error_mm(i); 
    relative_sum_dd_wo(i) = relative_sum_dd_wo(i-1) + relative_error_dd_wo(i); 
end

show_ind = 1:length(t_mm);

fig_1 = figure;
plot(t_mm(show_ind), relative_sum_mm(show_ind),...
        t_dd_wo(show_ind), relative_sum_dd_wo(show_ind),'LineWidth',2)
legend(parameter_set_mm, parameter_set_dd_wo, 'Location', 'northwest')
title('Integration of relative control error')
xlabel('time (s)')
ylabel('intergral of e^r')
saveas(fig_1, fig_path_integral_relative_error)






