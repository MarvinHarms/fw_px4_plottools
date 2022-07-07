function [outputArg1,outputArg2] = PathTrackingPlots(sysvector, topics)
%PATHTRACKINGPLOTS Summary of this function goes here
%   Detailed explanation goes here

% resample the position, commander state and gps to 10 Hz
min_time = realmin;
max_time = realmax;

min_time = max(min_time, sysvector.soaring_controller_position_0.pos_1.Time(1));
max_time = min(max_time, sysvector.soaring_controller_position_0.pos_1.Time(end));
time_resampled = min_time:0.1:max_time;

pos_lat = resample(sysvector.soaring_controller_position_0.pos_1, time_resampled);
pos_lon = resample(sysvector.soaring_controller_position_0.pos_0, time_resampled);
pos_alt = resample(sysvector.soaring_controller_position_0.pos_2, time_resampled);

pos_ref_lat = resample(sysvector.soaring_controller_position_setpoint_0.pos_1, time_resampled);
pos_ref_lon = resample(sysvector.soaring_controller_position_setpoint_0.pos_0, time_resampled);
pos_ref_alt = resample(sysvector.soaring_controller_position_setpoint_0.pos_2, time_resampled);

% only show plot, if 
if ~topics.commander_state.logged
    error('The commander_state topic needs to be logged')
end


% 3D plot of the global position estimate
if topics.soaring_controller_position_setpoint.logged
    fig1 = figure();
    fig1.Name = 'Estimated Position 3D Plot';
    % plot global position estimate
    plot3(double(pos_lon.Data(1:end)), ...
        double(pos_lat.Data(1:end)), ...
        double(pos_alt.Data(1:end)),...
        'Color', 'r', 'Linewidth', 1);
    hold on;
    plot3(double(pos_ref_lon.Data(1:end)), ...
        double(pos_ref_lat.Data(1:end)), ...
        double(pos_ref_alt.Data(1:end)),...
        'Color', 'b', 'Linewidth', 1);
    hold on;
    hold off;
    title('Global Position Estimate');

    hold off;
    colorbar
    xlabel('Latitude [m]');
    ylabel('Longitude [m]');
    zlabel('Altitude above MSL [m]');
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
figure;
plot(pos_ref_alt.Time(1:end),err);
xlabel('time [s]');
ylabel('abs tracking error [m]');
grid on;
title('position tracking error');


end

