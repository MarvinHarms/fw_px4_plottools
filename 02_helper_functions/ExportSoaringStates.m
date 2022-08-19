function ExportSoaringStates(sysvector, topics)
%PATHTRACKINGPLOTS Summary of this function goes here
%   Detailed explanation goes here
min_time = realmin;
max_time = realmax;

min_time = max(min_time, sysvector.soaring_controller_position_0.pos_1.Time(1)+1);
max_time = min(max_time, sysvector.soaring_controller_position_0.pos_1.Time(end)-1);
time_resampled = min_time:0.1:max_time;

%% true values
% position
pos_lat = resample(sysvector.soaring_controller_position_0.pos_1, time_resampled);
pos_lon = resample(sysvector.soaring_controller_position_0.pos_0, time_resampled);
pos_alt = resample(sysvector.soaring_controller_position_0.pos_2, time_resampled);
% velocity
vel_lat = resample(sysvector.soaring_controller_position_0.vel_1, time_resampled);
vel_lon = resample(sysvector.soaring_controller_position_0.vel_0, time_resampled);
vel_alt = resample(sysvector.soaring_controller_position_0.vel_2, time_resampled);
% attitude
q_0 = resample(sysvector.vehicle_attitude_0.q_0, time_resampled);
q_1 = resample(sysvector.vehicle_attitude_0.q_1, time_resampled);
q_2 = resample(sysvector.vehicle_attitude_0.q_2, time_resampled);
q_3 = resample(sysvector.vehicle_attitude_0.q_3, time_resampled);
% body rates
w_0 = time_resampled;   % not really needed, dummy input
w_1 = time_resampled;   % not really needed, dummy input
w_2 = time_resampled;   % not really needed, dummy input

%% reference values
% position
pos_ref_lat = resample(sysvector.soaring_controller_position_setpoint_0.pos_1, time_resampled);
pos_ref_lon = resample(sysvector.soaring_controller_position_setpoint_0.pos_0, time_resampled);
pos_ref_alt = resample(sysvector.soaring_controller_position_setpoint_0.pos_2, time_resampled);
% attitude
q_ref_0 = resample(sysvector.vehicle_attitude_setpoint_0.q_d_0, time_resampled);
q_ref_1 = resample(sysvector.vehicle_attitude_setpoint_0.q_d_1, time_resampled);
q_ref_2 = resample(sysvector.vehicle_attitude_setpoint_0.q_d_2, time_resampled);
q_ref_3 = resample(sysvector.vehicle_attitude_setpoint_0.q_d_3, time_resampled);


home_dir = '\\wsl$\ubuntu-20.04\home\marvin\Master_Thesis_ADS\Git_Python\master-thesis-ads\plotting\';
% export the sysvector in a form useable for the python plane animation:
ctrlX = [pos_lon.Data, pos_lat.Data, pos_alt.Data,...
        vel_lon.Data, vel_lat.Data, vel_alt.Data,...
        q_0.Data, q_1.Data, q_2.Data, q_3.Data,...
        w_0', w_1', w_2'];
save(append(home_dir,'ctrlX.mat'),'ctrlX');
    
simX = [pos_ref_lon.Data, pos_ref_lat.Data, pos_ref_alt.Data,...
        vel_lon.Data, vel_lat.Data, vel_alt.Data,...
        q_ref_0.Data, q_ref_1.Data, q_ref_2.Data, q_ref_3.Data,...
        w_0', w_1', w_2'];
save(append(home_dir,'simX.mat'),'simX');

end

