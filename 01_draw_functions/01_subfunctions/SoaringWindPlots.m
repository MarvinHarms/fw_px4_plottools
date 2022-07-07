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

    min_time = realmin;
    max_time = realmax;

    min_time = max(min_time, sysvector.wind_estimate_0.windspeed_east.Time(1)+1);
    max_time = min(max_time, sysvector.wind_estimate_0.windspeed_east.Time(end)-1);
    time_resampled = min_time:0.1:max_time;

    % plot the soaring wind estimate
    if reconstruct
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
        airspeed = resample(sysvector.airspeed_0.true_airspeed_m_s, time_resampled);
        % construct new timeseries
        wind_x = [];
        wind_y = [];
        wind_z = [];
        aoa = [];
        sideslip_force = [];
        % define transform
        R_ned_to_enu = zeros(3,3);
        R_ned_to_enu(1,2) = 1;
        R_ned_to_enu(2,1) = 1;
        R_ned_to_enu(3,3) = -1;
        for i=1:size(time_resampled,2)
            R_ib = R_ned_to_enu*R_ib_(:,:,i)';
            R_bi = R_ib';
            body_force = 1.4*R_bi*(R_ned_to_enu*[acc_0.Data(i);acc_1.Data(i);acc_2.Data(i)] + [0;0;9.81]);
            Fx = body_force(1);
            Fy = body_force(2);
            Fz = -body_force(3);
            speed = airspeed.Data(i);
            alpha = (((2*Fz)/(1.22*0.4*(max(speed*speed,9*9))+0.001) - 0.356)/2.354) / ...
                            (1 - ((2*Fx)/(1.22*0.4*(max(speed*speed,9*9))+0.001)/2.354));
            beta = 0;%0.1*Fy/speed;
            vel_air = R_ib*([speed;tan(beta)*speed;tan(alpha)*speed]);
            wind = R_ned_to_enu*[vel_0.Data(i);vel_1.Data(i);vel_2.Data(i)] - vel_air;
            aoa = [aoa, alpha];
            wind_x = [wind_x,wind(1)];
            wind_y = [wind_y,wind(2)];
            wind_z = [wind_z,wind(3)];
            sideslip_force = [sideslip_force,body_force(2)/1.4];
        end
        wind_x = timeseries(wind_x',time_resampled);
        wind_y = timeseries(wind_y',time_resampled);
        wind_z = timeseries(wind_z',time_resampled);
    else
        wind_y = resample(sysvector.soaring_controller_wind_0.wind_estimate_filtered_1, time_resampled);
        wind_x = resample(sysvector.soaring_controller_wind_0.wind_estimate_filtered_0, time_resampled);
        wind_z = resample(sysvector.soaring_controller_wind_0.wind_estimate_filtered_2, time_resampled);
    end
    
    % apply low-pass filter to wind estimates for smoothing
    cutoff_freq = 0.1;
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
    
    % plot the standard wind estimate
    wind_e = resample(sysvector.wind_estimate_0.windspeed_east, time_resampled);
    wind_n = resample(sysvector.wind_estimate_0.windspeed_north, time_resampled);
    
    % comppute total magnitude of the wind
    wind_magnitude_1 = sqrt(wind_x_filt.^2 + wind_y_filt.^2);
    wind_magnitude_2 = sqrt(wind_e.Data.^2 + wind_n.Data.^2);

    % compute leastsquares fit for lateral force coefficients
%     len = size(sideslip_filt,2);
%     A = -1*airspeed.Data'
%     y = sideslip_filt';
%     x = pinv(A'*A)*A'*y;
%     sideslip_fit = A*x;
    sideslip_fit = sideslip_filt./airspeed.Data;

    fig1 = figure();
    fig1.Name = 'Estimated Wind';
    plot1 = subplot(2,2,1);
    hold on;
    plot(time_resampled, wind_x.Data);
    plot(time_resampled, wind_x_filt);
    plot(time_resampled, wind_e.Data);
    xlabel('time (s)');
    ylabel('wind speed east (m/s)');
    legend('dynamic', 'dynamic filtered', 'EKF');
    grid on;
    plot2 = subplot(2,2,3);
    hold on;
    plot(time_resampled, wind_y.Data);
    plot(time_resampled, wind_y_filt);
    plot(time_resampled, wind_n.Data);
    xlabel('time (s)');
    ylabel('wind speed north (m/s)');
    legend('dynamic', 'dynamic filtered', 'EKF');
    grid on;
    plot3 = subplot(2,2,2);
    hold on;
    plot(time_resampled, wind_magnitude_1);
    plot(time_resampled, wind_magnitude_2);
    plot(time_resampled, airspeed.Data);
    xlabel('time (s)');
    ylabel('wind speed horizontal (m/s)');
    legend('dynamic', 'EKF', 'airspeed');
    grid on;
    plot4 = subplot(2,2,4);
    hold on;
    plot(time_resampled, sideslip_force,'LineWidth',1);
    plot(time_resampled, sideslip_filt);
    xlabel('time (s)');
    ylabel('lateral accel (m/s^2)');
    ylim([-2,2]);
    legend('raw', 'filtered');
    grid on;
    
    % link the time axis
    linkaxes([plot1,plot2,plot3,plot4],'x');

    
    

end

