function SoaringWindPlots(sysvector, topics)
%SOARINGWINDPLOTS Summary of this function goes here
%   Detailed explanation goes here
% resample the position, commander state and gps to 10 Hz
% check, if topics exist
    if ~topics.soaring_controller_wind.logged || ~topics.wind_estimate.logged
        disp('WIND ERROR');
        reconstruct = true;
    else
        reconstruct = true;
    end
    beta_scaling = 0.0;
    aoa_offset = 0.0;
    wing_area = 0.40;
    
    min_time = realmin;
    max_time = realmax;

    min_time = max(min_time, sysvector.wind_estimate_0.windspeed_east.Time(1)+1);
    max_time = min(max_time, sysvector.wind_estimate_0.windspeed_east.Time(end)-1);
    time_resampled = min_time:0.1:max_time;

    % plot the soaring wind estimate
    q_0 = resample(sysvector.vehicle_attitude_0.q_0, time_resampled);
    q_1 = resample(sysvector.vehicle_attitude_0.q_1, time_resampled);
    q_2 = resample(sysvector.vehicle_attitude_0.q_2, time_resampled);
    q_3 = resample(sysvector.vehicle_attitude_0.q_3, time_resampled);
    vel_0 = resample(sysvector.vehicle_local_position_0.vx, time_resampled);
    vel_1 = resample(sysvector.vehicle_local_position_0.vy, time_resampled);
    vel_2 = resample(sysvector.vehicle_local_position_0.vz, time_resampled);
    acc_0 = resample(sysvector.vehicle_local_position_0.ax, time_resampled);
    acc_1 = resample(sysvector.vehicle_local_position_0.ay, time_resampled);
    acc_2 = resample(sysvector.vehicle_local_position_0.az, time_resampled);
    R_ib_ = quat2dcm([q_0.Data,q_1.Data,q_2.Data,q_3.Data]);
    airspeed = resample(sysvector.airspeed_validated_0.true_airspeed_m_s, time_resampled);
    wind_x_online = resample(sysvector.soaring_controller_wind_0.wind_estimate_filtered_0, time_resampled);
    wind_y_online = resample(sysvector.soaring_controller_wind_0.wind_estimate_filtered_1, time_resampled);
    wind_z_online = resample(sysvector.soaring_controller_wind_0.wind_estimate_filtered_2, time_resampled);
    % construct new timeseries
    wind_x = [];
    wind_y = [];
    wind_z = [];
    aoa = [];
    C_l = [];
    sideslip_force = [];
    % define transform
    R_ned_to_enu = zeros(3,3);
    R_ned_to_enu(1,2) = 1;
    R_ned_to_enu(2,1) = 1;
    R_ned_to_enu(3,3) = -1;
    c_0 = 0.3; %0.356;
    c_1 = 2.3; %2.354;
    for i=1:size(time_resampled,2)
        R_ib = R_ned_to_enu*R_ib_(:,:,i)';
        R_bi = R_ib';
        body_force = 1.4*R_bi*(R_ned_to_enu*[acc_0.Data(i);acc_1.Data(i);acc_2.Data(i)] + [0;0;9.81]);
        % need to transform forces to wing frame
        Fx = cos(aoa_offset)*body_force(1) - sin(aoa_offset)*body_force(3);
        Fy = body_force(2);
        Fz =  - (sin(aoa_offset)*body_force(1) + cos(aoa_offset)*body_force(3));
        speed = (airspeed.Data(i))*1.0;
        c_l = -2*body_force(3)/(1.22*speed*speed*wing_area);
        alpha = (((2*Fz)/(1.22*wing_area*(max(speed*speed,9*9))+0.001) - c_0)/c_1) / ...
                        (1 - ((2*Fx)/(1.22*wing_area*(max(speed*speed,9*9))+0.001)/c_1));
        beta = beta_scaling*Fy;
        vel_air = R_ib*([speed;tan(beta)*speed;tan(alpha-aoa_offset)*speed]);
        wind = R_ned_to_enu*[vel_0.Data(i);vel_1.Data(i);vel_2.Data(i)] - vel_air;
        C_l = [C_l,c_l];
        aoa = [aoa, alpha];
        wind_x = [wind_x,wind(1)];
        wind_y = [wind_y,wind(2)];
        wind_z = [wind_z,wind(3)];
        sideslip_force = [sideslip_force,body_force(2)/1.4];
    end
    if reconstruct
        wind_x = timeseries(wind_x',time_resampled);
        wind_y = timeseries(wind_y',time_resampled);
        wind_z = timeseries(wind_z',time_resampled);
    else
        wind_y = resample(sysvector.soaring_controller_wind_0.wind_estimate_filtered_1, time_resampled);
        wind_x = resample(sysvector.soaring_controller_wind_0.wind_estimate_filtered_0, time_resampled);
        wind_z = resample(sysvector.soaring_controller_wind_0.wind_estimate_filtered_2, time_resampled);
    end
    
    % apply low-pass filter to wind estimates for smoothing
    cutoff_freq = 1;
    order = 2;
    %%%%%%%%%%%%%%
    Fs = size(wind_x.Time,1)/(wind_x.Time(end)-wind_x.Time(1));
    nyquist_freq = Fs/2;  % Nyquist frequency
    Wn=cutoff_freq/nyquist_freq;    % non-dimensional frequency
    [filtb,filta]=butter(order,Wn,'low'); % construct the filter
    %%%%%%%%%%%%%%
    wind_x_filt = filtfilt(filtb,filta,wind_x.Data);
    wind_y_filt = filtfilt(filtb,filta,wind_y.Data);
    wind_z_filt = filtfilt(filtb,filta,wind_z.Data);
    sideslip_filt = filtfilt(filtb,filta,sideslip_force);
    aoa_filt = filtfilt(filtb,filta,aoa);
    
    % plot the standard wind estimate
    wind_e = resample(sysvector.wind_estimate_0.windspeed_east, time_resampled);
    wind_n = resample(sysvector.wind_estimate_0.windspeed_north, time_resampled);
    
    % comppute total magnitude of the wind
    wind_magnitude_1 = sqrt(wind_x_filt.^2 + wind_y_filt.^2);
    wind_magnitude_2 = sqrt(wind_e.Data.^2 + wind_n.Data.^2);

    % compute leastsquares fit for wind triangle to determine wind and pitot scaling
    len = size(time_resampled,2);
    A = zeros(2*len,4);
    y = zeros(2*len,1);
    for i=1:len
        % compute velocities
        R_ib = R_ned_to_enu*R_ib_(:,:,i)';
        R_bi = R_ib';
        body_force = 1.4*R_bi*(R_ned_to_enu*[acc_0.Data(i);acc_1.Data(i);acc_2.Data(i)] + [0;0;9.81]);
        % need to transform forces to wing frame
        Fx = cos(aoa_offset)*body_force(1) - sin(aoa_offset)*body_force(3);
        Fy = body_force(2);
        Fz = -sin(aoa_offset)*body_force(1) - cos(aoa_offset)*body_force(3);
        speed = (airspeed.Data(i))*1;
        alpha = (((2*Fz)/(1.22*wing_area*(max(speed*speed,9*9))+0.001) - c_0)/c_1) / ...
                        (1 - ((2*Fx)/(1.22*wing_area*(max(speed*speed,9*9))+0.001)/c_1));
        beta = beta_scaling*Fy/speed;
        vel_air = R_ib*([speed;tan(beta)*speed;tan(alpha-aoa_offset)*speed]);
        vel_air_normalized = vel_air/sqrt(vel_air'*vel_air);
        % fill leastsquares matrix
        A(2*i-1,1) = vel_air(1);
        A(2*i,1) = vel_air(2);
        A(2*i-1,2) = vel_air_normalized(1);
        A(2*i,2) = vel_air_normalized(2);
        A(2*i-1,3) = 1;
        A(2*i,3) = 0;
        A(2*i-1,4) = 0;
        A(2*i,4) = 1;
        % fill leastsquares vector
        vel = R_ned_to_enu*[vel_0.Data(i);vel_1.Data(i);vel_2.Data(i)];
        y(2*i-1,1) = vel(1);
        y(2*i,1) = vel(2);
              
    end
    
    
    
    x = pinv(A'*A)*A'*y
    
    vel_magnitude = sqrt(vel_0.Data.^2 + vel_1.Data.^2 + vel_2.Data.^2);
    sideslip_fit = sideslip_filt./airspeed.Data;

    fig1 = figure();
    fig1.Name = 'Estimated Wind';
    plot1 = subplot(2,2,1);
    hold on;
    plot(time_resampled, wind_x_online.Data);
    plot(time_resampled, wind_x_filt);
    plot(time_resampled, wind_e.Data);
    xlabel('time (s)');
    ylabel('wind speed east (m/s)');
    legend('onboard','dynamic filtered', 'EKF');
    grid on;
    plot2 = subplot(2,2,3);
    hold on;
    plot(time_resampled, wind_y_online.Data);
    plot(time_resampled, wind_y_filt);
    plot(time_resampled, wind_n.Data);
    xlabel('time (s)');
    ylabel('wind speed north (m/s)');
    legend('onboard','dynamic filtered', 'EKF');
    grid on;
    plot3 = subplot(2,2,2);
    hold on;
%     plot(time_resampled, wind_magnitude_1);
%     plot(time_resampled, wind_magnitude_2);
%     xlabel('time (s)');
%     ylabel('wind speed horizontal (m/s)');
    plot(time_resampled, wind_z_online.Data);
    plot(time_resampled, wind_z_filt);
    xlabel('time (s)');
    ylabel('wind speed vertical (m/s)');
    legend('onboard','dynamic filtered');
    grid on;
    plot4 = subplot(2,2,4);
    hold on;
    plot(time_resampled, aoa_filt);
    xlabel('time (s)');
    ylabel('AoA (rad)');
    legend('filtered');
%     plot(time_resampled, sideslip_filt);
%     xlabel('time (s)');
%     ylabel('lateral accel (m/s^2)');
%     ylim([-2,2]);
    grid on;
    
    % link the time axis
    linkaxes([plot1,plot2,plot3,plot4],'x');
    hold off;

    
    % plot the difference in airspeed and inertial speed
    fig2 = figure();
    fig2.Name = 'airspeed scaling correctness';
    inertial_speed = sqrt(vel_0.Data.^2 + vel_1.Data.^2 + 0*vel_2.Data.^2);
    hold on;
    plot(time_resampled, inertial_speed);
    plot(time_resampled, airspeed.Data);
    legend('GPS', 'airspeed');
    grid on;
    hold off;
    

end

