%close all
close all
clear all

space_hold_1 = '%n';

%%%%%%%%%%% Stretching Analysis %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
file_error_dd_wo = 'stretching_status/BM/realtime_stretching_factor.txt';
file_error_mm = 'stretching_status/NM/realtime_stretching_factor.txt';

file_t_dd_wo = 'stretching_status/BM/control_time.txt';
file_t_mm = 'stretching_status/NM/control_time.txt';

fig_path_mm = 'stretching_status/realtime_stretching_factor.pdf';

parameter_set_dd_wo = 'B.M';
parameter_set_mm = 'N.M, s_s= 0.4';

stretching_factor_mm  = textread(file_error_mm, space_hold_1, 'headerlines',1);
stretching_factor_dd_wo  = textread(file_error_dd_wo, space_hold_1, 'headerlines',1);

[t_mm] = textread(file_t_mm, space_hold_1, 'headerlines',1);
t_dd_wo = textread(file_t_dd_wo, space_hold_1, 'headerlines',1);

%%%%%%%%%%%%%%%%%%%% show all in two plots: %%%%%%%%%%%%%%%%%%
fig_1 = figure;
plot(t_mm, stretching_factor_mm, t_dd_wo, stretching_factor_dd_wo, 'LineWidth',2)
legend(parameter_set_mm, parameter_set_dd_wo, 'Location', 'northwest')
title('stretching factor each step')
xlabel('time')
ylabel('stretching factor')
saveas(fig_1 ,fig_path_mm)



