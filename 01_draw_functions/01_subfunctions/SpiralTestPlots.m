function SpiralTestPlots(sysvector, topics, paramvector)
%SPIRALTESTPLOTS Summary of this function goes here
%   Detailed explanation goes here
min_time = realmin;
    max_time = realmax;

    min_time = max(min_time, sysvector.wind_estimate_0.windspeed_east.Time(1)+1);
    max_time = min(max_time, sysvector.wind_estimate_0.windspeed_east.Time(end)-1);
    time_resampled = min_time:0.2:max_time;
    len = size(time_resampled,2);

    % get the logged topics
    pos_0 = resample(sysvector.soaring_controller_position_0.pos_0, time_resampled);
    pos_1 = resample(sysvector.soaring_controller_position_0.pos_1, time_resampled);
    pos_2 = resample(sysvector.soaring_controller_position_0.pos_2, time_resampled);
    vel_0 = resample(sysvector.soaring_controller_position_0.vel_0, time_resampled);
    vel_1 = resample(sysvector.soaring_controller_position_0.vel_1, time_resampled);
    vel_2 = resample(sysvector.soaring_controller_position_0.vel_2, time_resampled);
    TAS = resample(sysvector.airspeed_validated_0.true_airspeed_m_s, time_resampled);
    inertial_speed = sqrt(vel_0.Data.^2 + vel_1.Data.^2 + vel_2.Data.^2);
    
    % actuators
    act_roll = resample(sysvector.actuator_controls_0_0.control_0, time_resampled);
    act_pitch = resample(sysvector.actuator_controls_0_0.control_1, time_resampled);
    
    % find saturation
    roll_sat = abs(act_roll.Data)>0.99;
    pitch_sat = abs(act_pitch.Data)>0.99;
    
    fig1 = figure();
    fig1.Name = 'airspeed-height XY-plot';
    %plot1 = subplot(2,1,1);
    hold on;
    

    % get the simulated response
    sim_runs = [".csv","_0_9.csv","_1_1.csv"];
    sim_styles = {'-k', '-.b', '-.c'};
    for i=1:3
        name_time = strcat('\\wsl$\Ubuntu-20.04\home\marvin\Master_Thesis_ADS\Git_Matlab\fw_px4_plottools\05_csv_files\spiral_time', sim_runs(i));
        name_pos = strcat('\\wsl$\Ubuntu-20.04\home\marvin\Master_Thesis_ADS\Git_Matlab\fw_px4_plottools\05_csv_files\spiral_pos', sim_runs(i));
        name_TAS = strcat('\\wsl$\Ubuntu-20.04\home\marvin\Master_Thesis_ADS\Git_Matlab\fw_px4_plottools\05_csv_files\spiral_airspeed', sim_runs(i));
        
        time_sim = table2array(readtable(name_time));
        pos_sim = table2array(readtable(name_pos));
        TAS_sim = table2array(readtable(name_TAS));
        pos_sim_0 = timeseries(pos_sim(1,1:end-1),time_sim);
        pos_sim_1 = timeseries(pos_sim(2,1:end-1),time_sim);
        pos_sim_2 = timeseries(pos_sim(3,1:end-1),time_sim);
        TAS_sim = timeseries(TAS_sim(1:end-1,:),time_sim);

        % resample simulation
        t_min = time_sim(1);
        t_max = time_sim(end);
        time_resampled = linspace(t_min,t_max,(t_max-t_min)*5);
        pos_sim_0 = resample(pos_sim_0,time_resampled).Data(:)';
        pos_sim_1 = resample(pos_sim_1,time_resampled).Data(:)';
        pos_sim_2 = resample(pos_sim_2,time_resampled).Data(:)';
        TAS_sim = resample(TAS_sim,time_resampled).Data;
        
        % plot perturbed trajec
        plot(pos_sim_2(:,:,:),TAS_sim, string(sim_styles(i)));
    end
   
    plot(pos_2.Data-14,TAS.Data,'--or');
    xlabel('height (m)');
    ylabel('TAS (m/s)');
    grid on;
    legend('simulation 1.0*$C_D$', 'simulation 0.9*$C_D$', 'simulation 1.1*$C_D$', 'experiment','Interpreter','latex');
    
%     plot1 = subplot(2,1,2);
%     hold on;
%     plot(pos_2.Data-15,roll_sat);
%     plot(pos_2.Data-15,pitch_sat);
%     xlabel('height (m)');
%     ylabel('inertial speed (m/s)');
%     grid on;
%     legend('experiment', 'simulation');
    
    fig2 = figure();
    fig2.Name = 'position tracking plot';
    plot3(-pos_1.Data,pos_0.Data,pos_2.Data-14,'r');
    hold on;
    plot3(pos_sim_0+20,pos_sim_1,pos_sim_2,'b');
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    daspect([1 1 1]);
    legend('experiment', 'simulation');
end

