function [outputArg1,outputArg2] = PathTrackingPlots(sysvector, topics)
%PATHTRACKINGPLOTS Summary of this function goes here
%   Detailed explanation goes here

% resample the position, commander state and gps to 10 Hz
min_time = realmin;
max_time = realmax;

min_time = max(min_time, sysvector.soaring_controller_position_0.pos_1.Time(1)+1);
max_time = min(max_time, sysvector.soaring_controller_position_0.pos_1.Time(end)-1);
time_resampled = min_time:0.01:max_time;

% airspeed
TAS = resample(sysvector.airspeed_validated_0.true_airspeed_m_s, time_resampled);

% position
pos_lat = resample(sysvector.soaring_controller_position_0.pos_1, time_resampled);
pos_lon = resample(sysvector.soaring_controller_position_0.pos_0, time_resampled);
pos_alt = resample(sysvector.soaring_controller_position_0.pos_2, time_resampled);
% velocity
vel_lat = resample(sysvector.soaring_controller_position_0.vel_1, time_resampled);
vel_lon = resample(sysvector.soaring_controller_position_0.vel_0, time_resampled);
vel_alt = resample(sysvector.soaring_controller_position_0.vel_2, time_resampled);
% acceleration
acc_lat = resample(sysvector.soaring_controller_position_0.acc_1, time_resampled);
acc_lon = resample(sysvector.soaring_controller_position_0.acc_0, time_resampled);
acc_alt = resample(sysvector.soaring_controller_position_0.acc_2, time_resampled);

% position
pos_ref_lat = resample(sysvector.soaring_controller_position_setpoint_0.pos_1, time_resampled);
pos_ref_lon = resample(sysvector.soaring_controller_position_setpoint_0.pos_0, time_resampled);
pos_ref_alt = resample(sysvector.soaring_controller_position_setpoint_0.pos_2, time_resampled);
% velocity
vel_ref_lat = resample(sysvector.soaring_controller_position_setpoint_0.vel_1, time_resampled);
vel_ref_lon = resample(sysvector.soaring_controller_position_setpoint_0.vel_0, time_resampled);
vel_ref_alt = resample(sysvector.soaring_controller_position_setpoint_0.vel_2, time_resampled);
% acceleration
acc_ref_lat = resample(sysvector.soaring_controller_position_setpoint_0.acc_1, time_resampled);
acc_ref_lon = resample(sysvector.soaring_controller_position_setpoint_0.acc_0, time_resampled);
acc_ref_alt = resample(sysvector.soaring_controller_position_setpoint_0.acc_2, time_resampled);

% only show plot, if 
if ~topics.commander_state.logged
    error('The commander_state topic needs to be logged')
end


% 3D plot of the global position estimate
if topics.soaring_controller_position_setpoint.logged
    fig1 = figure();
    fig1.Name = 'Estimated Position 3D Plot';
    % plot global position estimate
    h = scatter3(double(pos_lon.Data(1:end)), ...
        double(pos_lat.Data(1:end)), ...
        double(pos_alt.Data(1:end)),...
        3, TAS.data);
    hold on;
    plot3(double(pos_ref_lon.Data(1:end)), ...
        double(pos_ref_lat.Data(1:end)), ...
        double(pos_ref_alt.Data(1:end)),...
        '--k', 'Linewidth', 1);
    hold on;
    hold off;
    title('Local Position Estimate');

    hold off;
    h.MarkerFaceColor = 'flat';
    colormap('cool');
    c = colorbar;
    c.Label.String = 'true airspeed';
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    grid on
    
    % set axis limits
    daspect([1 1 1]);
%     min_lat = min(pos_lat.Data);
%     min_lon = min(pos_lon.Data);
%     max_lat = max(pos_lat.Data);
%     max_lon = max(pos_lon.Data);
%     diff_plot = max(max_lat-min_lat, max_lon-min_lon);
%     axis([min_lat+0.5*(max_lat-min_lat-diff_plot) min_lat+0.5*(max_lat-min_lat+diff_plot)...
%     min_lon+0.5*(max_lon-min_lon-diff_plot) min_lon+0.5*(max_lon-min_lon+diff_plot) -inf inf])
end



% compute tracking error
errX = double(pos_ref_lon.Data(1:end))- double(pos_lon.Data(1:end));
errY = double(pos_ref_lat.Data(1:end))- double(pos_lat.Data(1:end));
errZ = double(pos_ref_alt.Data(1:end))- double(pos_alt.Data(1:end));
err = sqrt(errX.^2 + errY.^2 + errZ.^2);
% compute velocity error
verrX = double(vel_ref_lon.Data(1:end))- double(vel_lon.Data(1:end));
verrY = double(vel_ref_lat.Data(1:end))- double(vel_lat.Data(1:end));
verrZ = double(vel_ref_alt.Data(1:end))- double(vel_alt.Data(1:end));
verr = sqrt(verrX.^2 + verrY.^2 + verrZ.^2);
% compute acceleration error
aerrX = double(acc_ref_lon.Data(1:end))- double(acc_lon.Data(1:end));
aerrY = double(acc_ref_lat.Data(1:end))- double(acc_lat.Data(1:end));
aerrZ = double(acc_ref_alt.Data(1:end))- double(acc_alt.Data(1:end));
aerr = sqrt(aerrX.^2 + aerrY.^2 + aerrZ.^2);
% some filtering
% apply low-pass filter to wind estimates for smoothing
cutoff_freq = 1;
order = 2;
%%%%%%%%%%%%%%
Fs = size(err,1)/(max_time-min_time);
nyquist_freq = Fs/2;  % Nyquist frequency
Wn=cutoff_freq/nyquist_freq;    % non-dimensional frequency
[filtb,filta]=butter(order,Wn,'low'); % construct the filter
%%%%%%%%%%%%%%
err = filtfilt(filtb,filta,err);
verr = filtfilt(filtb,filta,verr);
aerr = filtfilt(filtb,filta,aerr);



%plot
figure;
plot_1 = subplot(3,1,1);
plot(pos_ref_alt.Time(1:end),err);
xlabel('time [s]');
ylabel('error [m]');
grid on;
title('position tracking error');
plot_2 = subplot(3,1,2);
plot(pos_ref_alt.Time(1:end),verr);
xlabel('time [s]');
ylabel('error [$\frac{m}{s}$]','Interpreter','latex');
grid on;
title('velocity tracking error');
plot_3 = subplot(3,1,3);
plot(pos_ref_alt.Time(1:end),aerr);
xlabel('time [s]');
ylabel('error [$\frac{m}{s^2}$]','Interpreter','latex');
grid on;
title('acceleration tracking error');

% link the time axis
linkaxes([plot_1,plot_2,plot_3],'x');




end

