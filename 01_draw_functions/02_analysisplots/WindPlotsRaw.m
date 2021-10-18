function [mean_raw_wind, tspan1] = WindPlotsRaw(sysvector, topics, paramvector, params, config)
% Display the wind data.

tspan = [config.t_st_cal, config.t_ed_cal];

% get the aoa and slip angles if logged
if config.use_airflow_angles
    [aoa, slip, aoa_logged, slip_logged] = CalibrateAirflowAngles(sysvector, topics, paramvector, params, config);
else
    aoa_logged = false;
    slip_logged = false;
end

figure('color', 'w', 'name', 'Reconstructed Wind 3D');
% synchronise the data
dt = 0.05;
min_time = max([tspan(1), ...
    sysvector.airspeed_0.true_airspeed_m_s.Time(1),...
    sysvector.sensor_gyro_0.x.Time(1),...
    sysvector.airflow_aoa_0.aoa_rad.Time(1),...
    sysvector.airflow_slip_0.slip_rad.Time(1),...
    sysvector.vehicle_attitude_0.q_0.Time(1),...
    sysvector.vehicle_local_position_0.x.Time(1)]);
max_time = min([tspan(2), ...
    sysvector.airspeed_0.true_airspeed_m_s.Time(end),...
    sysvector.sensor_gyro_0.x.Time(end),...
    sysvector.airflow_aoa_0.aoa_rad.Time(end),...
    sysvector.airflow_slip_0.slip_rad.Time(end),...
    sysvector.vehicle_attitude_0.q_0.Time(end),...
    sysvector.vehicle_local_position_0.x.Time(end)]);

if (aoa_logged)
    min_time = max([min_time, aoa.Time(1)]);
    max_time = min([max_time, aoa.Time(end)]);
end

if (slip_logged)
    min_time = max([min_time, slip.Time(1)]);
    max_time = min([max_time, slip.Time(end)]);
end

time_resampled = min_time:dt:max_time;
tspan1 = [min_time, max_time];

vel_n = resample(sysvector.vehicle_local_position_0.vx, time_resampled);
vel_e = resample(sysvector.vehicle_local_position_0.vy, time_resampled);
vel_d = resample(sysvector.vehicle_local_position_0.vz, time_resampled);
airspeed = resample(sysvector.airspeed_0.true_airspeed_m_s, time_resampled);
if aoa_logged
    aoa = resample(aoa, time_resampled);
else
    aoa = timeseries(zeros(size(time_resampled')), time_resampled);
end
if slip_logged
    slip = resample(slip, time_resampled);
else
    slip = timeseries(zeros(size(time_resampled')), time_resampled);
end
q_0 = resample(sysvector.vehicle_attitude_0.q_0, time_resampled);
q_1 = resample(sysvector.vehicle_attitude_0.q_1, time_resampled);
q_2 = resample(sysvector.vehicle_attitude_0.q_2, time_resampled);
q_3 = resample(sysvector.vehicle_attitude_0.q_3, time_resampled);
if topics.sensor_gyro.logged
    gyro_x = resample(sysvector.sensor_gyro_0.x, time_resampled);
    gyro_y = resample(sysvector.sensor_gyro_0.y, time_resampled);
    gyro_z = resample(sysvector.sensor_gyro_0.z, time_resampled);
else
    gyro_x = timeseries(zeros(size(time_resampled)), time_resampled);
    gyro_y = timeseries(zeros(size(time_resampled)), time_resampled);
    gyro_z = timeseries(zeros(size(time_resampled)), time_resampled);
end

R_I_B = quat2rotm([q_0.Data, q_1.Data, q_2.Data, q_3.Data]);



% reconstructed wind
if (isfield(config, 'slip_offset_x'))
   slip_offset_x = config.slip_offset_x; 
else
    slip_offset_x = 0.0;
end
if (isfield(config, 'slip_offset_z'))
   slip_offset_z = config.slip_offset_z; 
else
    slip_offset_z = 0.0;
end
if (isfield(config, 'aoa_offset_x'))
   aoa_offset_x = config.aoa_offset_x; 
else
    aoa_offset_x = 0.0;
end
if (isfield(config, 'aoa_offset_y'))
   aoa_offset_y = config.aoa_offset_y; 
else
    aoa_offset_y = 0.0;
end
v_air_body = [ ...
    airspeed.Data'; ...
    (airspeed.Data .* tan(slip.Data) - slip_offset_x * gyro_z.Data + slip_offset_z * gyro_x.Data)'; ...
    (airspeed.Data .* tan(aoa.Data) + aoa_offset_x * gyro_y.Data - aoa_offset_y * gyro_x.Data)'];

v_air = [...
    sum(squeeze(R_I_B(1,:,:)) .* v_air_body, 1); ...
    sum(squeeze(R_I_B(2,:,:)) .* v_air_body, 1); ...
    sum(squeeze(R_I_B(3,:,:)) .* v_air_body, 1); ...    
    ];

v_gnd = [vel_n.Data'; vel_e.Data'; vel_d.Data'];
wind = v_gnd - v_air;

if ~aoa_logged
    % if the aoa is not logged set the vertical wind to 0
    wind(3, :) = 0.0 * wind(3, :);
end


% calculate mean wind components
mean_raw_wind = mean(wind, 2);

% / position plot / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
if (topics.vehicle_local_position.logged)
    pos_x = resample(sysvector.vehicle_local_position_0.x, time_resampled);
    pos_y = resample(sysvector.vehicle_local_position_0.y, time_resampled);
    pos_z = resample(sysvector.vehicle_local_position_0.z, time_resampled);
    
    % make z up
    pos_z.Data = -pos_z.Data;
    vel_d.Data = -vel_d.Data;

    plot3(pos_x.Data,pos_y.Data,pos_z.Data,'k.');
    axis equal
    hold all
    text(pos_x.Data(1),pos_y.Data(1),pos_z.Data(1),'t_{min}');
    text(pos_x.Data(end),pos_y.Data(end),pos_z.Data(end),'t_{max}');
    di = numel(pos_x.Data);
    distep = floor(di/20);
    for i=1:20
        text(pos_x.Data(i*distep), ...
            pos_y.Data(i*distep), ...
            pos_z.Data(i*distep), ...
            num2str(pos_x.Time(i*distep,1), '%.1f'));
    end

    quiver3(pos_x.Data, pos_y.Data, pos_z.Data,...
            wind(2, :)', wind(1, :)', wind(3, :)', 0);
    legend('pos','v_{wind}');
    title('Wind Data');
    xlabel('delta-Longitude [m]');
    ylabel('delta-Latitude [m]');
    zlabel('Altitude above MSL [m]');
    grid on
end

% / wind plot / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
figure('color','w','name','Reconstructed Wind Plot');

if (aoa_logged || slip_logged)
    num_plots = 4;
else
    num_plots = 3;
end
wind_plot(1) = subplot(num_plots,1,1); hold on; grid on; box on;
plot(time_resampled, wind(2, :));
plot(time_resampled, wind(1, :));
plot(time_resampled, wind(3, :));
plot(time_resampled([1 end]), mean_raw_wind(2) * ones(1,2), 'linewidth', 2);
plot(time_resampled([1 end]), mean_raw_wind(1) * ones(1,2), 'linewidth', 2);
plot(time_resampled([1 end]), mean_raw_wind(3) * ones(1,2), 'linewidth', 2);
legend('wind east', 'wind north', 'wind down', 'wind east mean', 'wind north mean', 'wind down mean');
ylabel('Wind Components [m/s]');

wind_plot(2) = subplot(num_plots,1,2); hold on; grid on; box on;
plot(time_resampled, sqrt(vel_n.Data.^2 + vel_e.Data.^2));
plot(time_resampled, airspeed.Data);
plot(time_resampled, sqrt(wind(1, :).^2 + wind(2, :).^2));
ylabel('Speeds [m/s]');
legend('ground sp.', 'airspeed', 'wind sp.');

wind_plot(3) = subplot(num_plots,1,3); hold on; grid on; box on;
if (topics.vehicle_local_position.logged)
    plot(time_resampled, pos_z.Data);
    ylabel('Height [m]');
else
    text((time_resampled(end)-time_resampled(1))/2+time_resampled(1), 0, 'no position data logged', ...
        'fontsize', 14, 'horizontalalignment', 'center');
    ylim([-1, 1]);
end

if (aoa_logged || slip_logged)
    wind_plot(4) = subplot(num_plots,1,4); hold on; grid on; box on;
    ylabel('Airflow Angles [deg]');
    plot(time_resampled, rad2deg(aoa.Data));
    plot(time_resampled, rad2deg(slip.Data));
    legend('aoa', 'slip');
end

xlabel('Time [s]');
linkaxes(wind_plot(:),'x');
xlim(wind_plot(:), time_resampled([1 end]));
end

