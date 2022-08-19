function AttitudeTrackingPlots(sysvector, topics)
%ATTITUDETRACKINGPLOTS Summary of this function goes here
%   Detailed explanation goes here
    if ~topics.vehicle_angular_acceleration.logged || ~topics.wind_estimate.logged
        return;
    end

    min_time = realmin;
    max_time = realmax;

    min_time = max(min_time, sysvector.vehicle_angular_acceleration_0.xyz_0.Time(1)+1);
    max_time = min(max_time, sysvector.vehicle_angular_acceleration_0.xyz_0.Time(end)-1);
    time_resampled = min_time:0.01:max_time;

    acc_roll = resample(sysvector.vehicle_angular_acceleration_0.xyz_0, time_resampled);
    acc_pitch = resample(sysvector.vehicle_angular_acceleration_0.xyz_1, time_resampled);
    acc_yaw = resample(sysvector.vehicle_angular_acceleration_0.xyz_2, time_resampled);
    acc_ref_roll = resample(sysvector.vehicle_angular_acceleration_setpoint_0.xyz_0, time_resampled);
    acc_ref_pitch = resample(sysvector.vehicle_angular_acceleration_setpoint_0.xyz_1, time_resampled);
    acc_ref_yaw = resample(sysvector.vehicle_angular_acceleration_setpoint_0.xyz_2, time_resampled);
    act_roll = resample(sysvector.actuator_controls_0_0.control_0, time_resampled);
    act_pitch = resample(sysvector.actuator_controls_0_0.control_1, time_resampled);
    act_yaw = resample(sysvector.actuator_controls_0_0.control_2, time_resampled);


    
    % apply low-pass filter to wind estimates for smoothing
    cutoff_freq = 6;
    order = 2;
    %%%%%%%%%%%%%%
    Fs = size(acc_roll.Time,1)/(acc_roll.Time(end)-acc_roll.Time(1));
    nyquist_freq = Fs/2;  % Nyquist frequency
    Wn=cutoff_freq/nyquist_freq;    % non-dimensional frequency
    [filtb,filta]=butter(order,Wn,'low'); % construct the filter
    %%%%%%%%%%%%%%
    acc_roll_filt = filtfilt(filtb,filta,acc_roll.Data);
    acc_pitch_filt = filtfilt(filtb,filta,acc_pitch.Data);
    acc_yaw_filt = filtfilt(filtb,filta,acc_yaw.Data);
    acc_ref_roll_filt = filtfilt(filtb,filta,acc_ref_roll.Data);
    acc_ref_pitch_filt = filtfilt(filtb,filta,acc_ref_pitch.Data);
    acc_ref_yaw_filt = filtfilt(filtb,filta,acc_ref_yaw.Data);


    fig1 = figure();
    fig1.Name = 'Angualar Acceleration';
    plot1 = subplot(3,1,1);
    hold on;
    %plot(time_resampled, wind_x.Data);
    plot(time_resampled, acc_roll_filt);
    plot(time_resampled, acc_ref_roll.Data);
    plot(time_resampled, acc_ref_roll_filt);
    xlabel('time (s)');
    ylabel('roll acceleration (m/s)');
    legend('true', 'ref', 'ref filt');
    grid on;
    plot2 = subplot(3,1,2);
    hold on;
    %plot(time_resampled, wind_y.Data);
    plot(time_resampled, acc_pitch_filt);
    plot(time_resampled, acc_ref_pitch.Data);
    plot(time_resampled, acc_ref_pitch_filt);
    xlabel('time (s)');
    ylabel('pitch acceleration (m/s)');
    legend('true', 'ref', 'ref filt');
    grid on;
    plot3 = subplot(3,1,3);
    hold on;
    plot(time_resampled, act_roll.Data);
    plot(time_resampled, act_pitch.Data);
    plot(time_resampled, act_yaw.Data);
    xlabel('time (s)');
    ylabel('actuator deflections (m/s)');
    legend('ailerons', 'elevator','rudder');
    grid on;

    % link the time axis
    linkaxes([plot1,plot2,plot3],'x');

end

