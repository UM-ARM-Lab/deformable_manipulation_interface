%close all
close all
clear all

space_hold_1 = '%n';

%%%%%%%%%%% Stretching Analysis %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
file_stretching_dd_wo = 'stretching_status/BM/realtime_stretching_factor.txt';
file_stretching_mm_04 = 'stretching_status/NM/cos_04/realtime_stretching_factor.txt';
file_stretching_mm_06 = 'stretching_status/NM/cos_06/realtime_stretching_factor.txt';
file_stretching_mm_08 = 'stretching_status/NM/cos_08/realtime_stretching_factor.txt';

file_t_dd_wo = 'stretching_status/BM/control_time.txt';
file_t_mm_04 = 'stretching_status/NM/cos_04/control_time.txt';
file_t_mm_06 = 'stretching_status/NM/cos_06/control_time.txt';
file_t_mm_08 = 'stretching_status/cloth_single_pole/NM/cos_08/control_time.txt';

fig_path_mm = 'stretching_status/cloth_single_pole/realtime_stretching_factor.pdf';

parameter_set_dd_wo = 'B.M';
parameter_set_mm_04 = 'N.M, s_s= 0.4';
parameter_set_mm_06 = 'N.M, s_s= 0.6';
parameter_set_mm_08 = 'N.M, s_s= 0.8';

stretching_factor_mm_04  = textread(file_stretching_mm_04, space_hold_1, 'headerlines',1);
stretching_factor_mm_06  = textread(file_stretching_mm_06, space_hold_1, 'headerlines',1);
stretching_factor_mm_08  = textread(file_stretching_mm_08, space_hold_1, 'headerlines',1);
stretching_factor_dd_wo  = textread(file_stretching_dd_wo, space_hold_1, 'headerlines',1);

t_mm_04 = textread(file_t_mm_04, space_hold_1, 'headerlines',1);
t_mm_06 = textread(file_t_mm_06, space_hold_1, 'headerlines',1);
t_mm_08 = textread(file_t_mm_08, space_hold_1, 'headerlines',1);
t_dd_wo = textread(file_t_dd_wo, space_hold_1, 'headerlines',1);

%%%%%%%%%%%%%%%%%%%% show all in two plots: %%%%%%%%%%%%%%%%%%
fig_1 = figure;
plot(t_mm_04, stretching_factor_mm_04,...
    t_mm_06, stretching_factor_mm_06,...
    t_mm_08, stretching_factor_mm_08,...
    t_dd_wo, stretching_factor_dd_wo, 'LineWidth',2)
legend(parameter_set_mm_04, parameter_set_mm_06,...
    parameter_set_mm_08, parameter_set_dd_wo,...
    'Location', 'northwest')
title('stretching factor each step')
xlabel('time')
ylabel('stretching factor')
saveas(fig_1 ,fig_path_mm)



