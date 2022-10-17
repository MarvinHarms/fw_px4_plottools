function SoaringWindPlots(sysvector, topics, paramvector)
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
    
    %% Parameter switches
    solve_inertial_calibration = false;
    solve_vane_calibration = false;
    vane_measurements = false;

    
    %% Calibration parameters for wind computation (inertial wind estimate)
    % params were optained by nonlinear optimization of the wind triangle
    % of flight log 'log_33_2022-7-28-12-55-42' in time interval [530,600]
    % seconds
    mass = 1.35;
    aoa_offset = 0.07;
    wing_area = 0.40;
    c_0 = 0.3; %0.3564, from semester project GA. Heinrich;
    c_1 = 2.354; %2.354, from semester project GA. Heinrich;
    rho = 1.22; % standard air density at sea level
    aspd_scale_calib = 1.0%0.9570;
    c_a0_calib = 0.19;%0.2798;
    c_a1_calib = 3.6;%4.3138;
    c_b1_calib = -3.4646;
    elev_eff_calib = 0.0;
    
    %% Calibration parameters for wind computation (vane wind estimate)
    aoa_bias_calib = -0.0311;
    slip_bias_calib = 0.0438;
    
%%    calibration log_33_2022-7-28-12-55-42' in time interval [530,600]:
%     mass = 1.35;
%     aoa_offset = 0.07;
%     wing_area = 0.40;
%     rho = 1.22;
%     aspd_scale_calib = 1.0839; (ASPD_SCALE was 1.15)
%     c_a0_calib = 0.2204;
%     c_a1_calib = 2.354;
%     c_b1_calib = -5.6789;

%%    calibration log_69_2022-9-8-17-54-32 in time interval [135, 288]:
%     aspd_scale_calib = 0.9984; (ASPD_SCALE was 1.16);
%     c_a0_calib = 0.2699;
%     c_a1_calib = 7.9485;
%     c_b1_calib = -3.4646;
    
    min_time = realmin;
    max_time = realmax;

    min_time = max(min_time, sysvector.wind_estimate_0.windspeed_east.Time(1)+1);
    max_time = min(max_time, sysvector.wind_estimate_0.windspeed_east.Time(end)-1);
    time_resampled = min_time:0.2:max_time;
    len = size(time_resampled,2);

    % plot the soaring wind estimate
    q_0 = resample(sysvector.vehicle_attitude_0.q_0, time_resampled);
    q_1 = resample(sysvector.vehicle_attitude_0.q_1, time_resampled);
    q_2 = resample(sysvector.vehicle_attitude_0.q_2, time_resampled);
    q_3 = resample(sysvector.vehicle_attitude_0.q_3, time_resampled);
    pos_0 = resample(sysvector.vehicle_local_position_0.x, time_resampled);
    pos_1 = resample(sysvector.vehicle_local_position_0.y, time_resampled);
    pos_2 = resample(sysvector.vehicle_local_position_0.z, time_resampled);
    vel_0 = resample(sysvector.vehicle_local_position_0.vx, time_resampled);
    vel_1 = resample(sysvector.vehicle_local_position_0.vy, time_resampled);
    vel_2 = resample(sysvector.vehicle_local_position_0.vz, time_resampled);
    acc_0 = resample(sysvector.vehicle_local_position_0.ax, time_resampled);
    acc_1 = resample(sysvector.vehicle_local_position_0.ay, time_resampled);
    acc_2 = resample(sysvector.vehicle_local_position_0.az, time_resampled);
    R_ib_ = quat2dcm([q_0.Data,q_1.Data,q_2.Data,q_3.Data]);
    IAS = resample(sysvector.airspeed_validated_0.indicated_airspeed_m_s, time_resampled);
    CAS = resample(sysvector.airspeed_validated_0.calibrated_airspeed_m_s, time_resampled);
    TAS = resample(sysvector.airspeed_validated_0.true_airspeed_m_s, time_resampled);
    wind_x_online = resample(sysvector.soaring_controller_wind_0.wind_estimate_filtered_0, time_resampled);
    wind_y_online = resample(sysvector.soaring_controller_wind_0.wind_estimate_filtered_1, time_resampled);
    wind_z_online = resample(sysvector.soaring_controller_wind_0.wind_estimate_filtered_2, time_resampled);
    elev_deflection = resample(sysvector.actuator_controls_0_0.control_1, time_resampled);
    
    % define transform
    R_ned_to_enu = zeros(3,3);
    R_ned_to_enu(1,2) = 1;
    R_ned_to_enu(2,1) = 1;
    R_ned_to_enu(3,3) = -1;
    
    %% solve nonlinear optimization problem on aerodynamic coefficients
    % An optimal control problem (OCP),
    % solved with direct multiple-shooting.
    if (solve_inertial_calibration)
       
        N = len-1; % number of control intervals
        opti = casadi.Opti(); % Optimization problem

        % ---- decision variables ---------
        c_a0 = opti.variable();
        c_a1 = opti.variable();
        c_b1 = opti.variable();
        scale = opti.variable();    %airspeed scaling
        V_a = opti.variable(3,N+1); % air relative velocity in body frame (true airspeed)
        V_g = opti.variable(3,N+1); % ground velocity in NED frame
        W = opti.variable(3,1); % wind
        elev_eff = opti.variable(); % elevator effectivity

        % ---- objective          ---------
        cost = 0;
        for i=1:len
            % cost is the error in the wind triangle
            R_ib = R_ned_to_enu*R_ib_(:,:,i)';
            cost = cost + norm(R_ib*V_a(:,i) + W(:,1) - R_ned_to_enu*V_g(:,i));
        end
        opti.minimize(cost); 

        % ------- constraints -------------
        for i=1:len
            %
            R_ib = R_ned_to_enu*R_ib_(:,:,i)';
            R_bi = R_ib';
            body_force = mass*R_bi*(R_ned_to_enu*[acc_0.Data(i);acc_1.Data(i);acc_2.Data(i)] + [0;0;9.81]);

            % set ground velocity
            opti.subject_to(V_g(1,i)==vel_0.Data(i));
            opti.subject_to(V_g(2,i)==vel_1.Data(i));
            opti.subject_to(V_g(3,i)==vel_2.Data(i));
            % set body velocity
            t_speed = scale*TAS.Data(i);  % true airspeed
            c_speed = scale*CAS.Data(i); % calibrated airspeed
            elev_force = 0.5*rho*(c_speed^2)*elev_eff*elev_deflection.Data(i)/1.0;  % elevator force
            opti.subject_to(V_a(1,i)==t_speed);
            opti.subject_to(0.5*rho*(c_speed^2)*wing_area*c_b1*V_a(2,i)/t_speed==body_force(2));
            %opti.subject_to(V_a(0,i)==0);
            opti.subject_to(0.5*rho*(c_speed^2)*wing_area*(c_a0 + c_a1*V_a(3,i)/t_speed)-elev_force==-body_force(3));
        end

        opti.subject_to(W(3,1)==0);
        opti.subject_to(c_b1==c_b1_calib);
        %opti.subject_to(c_a0==c_0);
        opti.subject_to(c_a1==c_a1_calib);
        opti.subject_to(scale==aspd_scale_calib);
        opti.subject_to(elev_eff==0);
        %opti.subject_to(elev_eff>=0);
        %opti.subject_to(elev_eff<=0.2);

        % ---- initial values for solver ---
        opti.set_initial(c_a0, c_0);
        opti.set_initial(c_a1, c_1);
        opti.set_initial(c_b1, 0.0);
        opti.set_initial(scale, 1.0);
        opti.set_initial(elev_eff, 0);
        for i=1:N+1
            opti.set_initial(V_a(1,i),TAS.Data(i));
            opti.set_initial(V_a(2,i),0);
            opti.set_initial(V_a(3,i),0);
            opti.set_initial(V_g(1,i),vel_0.Data(i));
            opti.set_initial(V_g(2,i),vel_1.Data(i));
            opti.set_initial(V_g(3,i),vel_2.Data(i));
        end
        opti.set_initial(W(1,1),0);
        opti.set_initial(W(2,1),0);
        opti.set_initial(W(3,1),0);

        % solve opti
        opti.solver('ipopt'); % set numerical backend
        sol = opti.solve();   % actual solve
        c_a0_calib = sol.value(c_a0)
        c_a1_calib = sol.value(c_a1)
        c_b1_calib = sol.value(c_b1)
        aspd_scale_calib = sol.value(scale)
        elev_eff_calib = sol.value(elev_eff)
        sol.value(W)
        
    end
    
    
    %% read vane measurements if present
    if (vane_measurements)
        %
        aoa_meas = resample(sysvector.airflow_aoa_0.aoa_rad, time_resampled);
        slip_meas = resample(sysvector.airflow_slip_0.slip_rad, time_resampled);
        aoa_meas.Data = -1*aoa_meas.Data;
    end
    
    %% solve nonlinear optimization problem for vane offsets
    % An optimal control problem (OCP),
    % solved with direct multiple-shooting.
    if (solve_vane_calibration)
       
        N = len-1; % number of control intervals
        opti = casadi.Opti(); % Optimization problem

        % ---- decision variables ---------
        aoa_bias = opti.variable();
        slip_bias = opti.variable();
        scale = opti.variable();    %airspeed scaling
        V_a = opti.variable(3,N+1); % air relative velocity in body frame (true airspeed)
        V_g = opti.variable(3,N+1); % ground velocity in NED frame
        W = opti.variable(3,1); % wind

        % ---- objective          ---------
        cost = 0;
        for i=1:len
            % cost is the error in the wind triangle
            R_ib = R_ned_to_enu*R_ib_(:,:,i)';
            cost = cost + norm(R_ib*V_a(:,i) + W(:,1) - R_ned_to_enu*V_g(:,i));
        end
        opti.minimize(cost); 

        % ------- constraints -------------
        for i=1:len
            %
            R_ib = R_ned_to_enu*R_ib_(:,:,i)';
            R_bi = R_ib';

            % set ground velocity
            opti.subject_to(V_g(1,i)==vel_0.Data(i));
            opti.subject_to(V_g(2,i)==vel_1.Data(i));
            opti.subject_to(V_g(3,i)==vel_2.Data(i));
            % set body velocity
            t_speed = scale*TAS.Data(i);  % true airspeed
            opti.subject_to(V_a(1,i)==t_speed);
            opti.subject_to(V_a(2,i)==t_speed*tan(slip_meas.Data(i)-slip_bias));
            opti.subject_to(V_a(3,i)==t_speed*tan(aoa_meas.Data(i)-aoa_bias));
        end

        opti.subject_to(W(3,1)==0);
        %opti.subject_to(aoa_bias==aoa_bias_calib);
        %opti.subject_to(slip_bias==slip_bias_calib);
        %opti.subject_to(scale==aspd_scale_calib);

        % ---- initial values for solver ---
        opti.set_initial(aoa_bias, 0);
        opti.set_initial(slip_bias, 0);
        opti.set_initial(scale, 1.0);
        for i=1:N+1
            opti.set_initial(V_a(1,i),TAS.Data(i));
            opti.set_initial(V_a(2,i),0);
            opti.set_initial(V_a(3,i),0);
            opti.set_initial(V_g(1,i),vel_0.Data(i));
            opti.set_initial(V_g(2,i),vel_1.Data(i));
            opti.set_initial(V_g(3,i),vel_2.Data(i));
        end
        opti.set_initial(W(1,1),0);
        opti.set_initial(W(2,1),0);
        opti.set_initial(W(3,1),0);

        % solve opti
        opti.solver('ipopt'); % set numerical backend
        sol = opti.solve();   % actual solve
        aoa_bias_calib = sol.value(aoa_bias)
        slip_bias_calib = sol.value(slip_bias)
        aspd_scale_calib = sol.value(scale)
        sol.value(W)
        
    end
    
    %% compute reconstructed wind states ()
    % construct new timeseries
    wind_x = [];
    wind_y = [];
    wind_z = [];
    wind_x_vane = [];
    wind_y_vane = [];
    wind_z_vane = [];
    aoa = [];
    slip = [];
    aoa_vane = [];
    C_l = [];
    sideslip_force = [];
 
    for i=1:size(time_resampled,2)
        R_ib = R_ned_to_enu*R_ib_(:,:,i)';
        R_bi = R_ib';
        body_force = mass*R_bi*(R_ned_to_enu*[acc_0.Data(i);acc_1.Data(i);acc_2.Data(i)] + [0;0;9.81]);
        % compute inertial wind estimate
        Fx = cos(aoa_offset)*body_force(1) - sin(aoa_offset)*body_force(3);
        Fy = body_force(2);
        Fz =  - (sin(aoa_offset)*body_force(1) + cos(aoa_offset)*body_force(3));
        speed = (TAS.Data(i))*1.21/1.21;
        c_l = -2*body_force(3)/(rho*speed*speed*wing_area);
        alpha = (((2*Fz)/(rho*wing_area*(max(speed*speed,9*9))+0.001) - c_a0_calib)/c_a1_calib) / ...
                        (1 - ((2*Fx)/(rho*wing_area*(max(speed*speed,9*9))+0.001)/c_a1_calib));
        speed = aspd_scale_calib*TAS.Data(i);
        t_speed = aspd_scale_calib*TAS.Data(i);  % true airspeed
        c_speed = aspd_scale_calib*CAS.Data(i); % calibrated airspeed
        elev_force = 0.5*rho*(c_speed^2)*elev_eff_calib*elev_deflection.Data(i)/1.0;  % elevator force
        u = t_speed;
        v = body_force(2)/(0.5*rho*(c_speed^2)*wing_area*c_b1_calib)*t_speed;
        w = ((-body_force(3)+elev_force)/(0.5*rho*(c_speed^2)*wing_area)-c_a0_calib)*t_speed/c_a1_calib;
        vel_air = R_ib*[u;v;w];
        wind = R_ned_to_enu*[vel_0.Data(i);vel_1.Data(i);vel_2.Data(i)] - vel_air;
        C_l = [C_l,c_l];
        aoa = [aoa, alpha];
        slip = [slip, tan(v/u)];
        wind_x = [wind_x,wind(1)];
        wind_y = [wind_y,wind(2)];
        wind_z = [wind_z,wind(3)];
        sideslip_force = [sideslip_force,body_force(2)/mass];
        % compute vane wind estimate
        if vane_measurements
            u = t_speed;
            v = tan(slip_meas.Data(i)-slip_bias_calib)*t_speed;
            w = tan(aoa_meas.Data(i)-aoa_bias_calib)*t_speed;
            vel_air = R_ib*[u;v;w];
            wind_vane = R_ned_to_enu*[vel_0.Data(i);vel_1.Data(i);vel_2.Data(i)] - vel_air;
            wind_x_vane = [wind_x_vane,wind_vane(1)];
            wind_y_vane = [wind_y_vane,wind_vane(2)];
            wind_z_vane = [wind_z_vane,wind_vane(3)];
        end
    end
    if reconstruct
        wind_x = timeseries(wind_x',time_resampled);
        wind_y = timeseries(wind_y',time_resampled);
        wind_z = timeseries(wind_z',time_resampled);
        if vane_measurements
            wind_x_vane = timeseries(wind_x_vane',time_resampled);
            wind_y_vane = timeseries(wind_y_vane',time_resampled);
            wind_z_vane = timeseries(wind_z_vane',time_resampled);
        end
    else
        wind_y = resample(sysvector.soaring_controller_wind_0.wind_estimate_filtered_1, time_resampled);
        wind_x = resample(sysvector.soaring_controller_wind_0.wind_estimate_filtered_0, time_resampled);
        wind_z = resample(sysvector.soaring_controller_wind_0.wind_estimate_filtered_2, time_resampled);
    end
    
    % apply low-pass filter to wind estimates for smoothing
    cutoff_freq = 1.0;
    order = 2;
    %%%%%%%%%%%%%%
    Fs = size(wind_x.Time,1)/(wind_x.Time(end)-wind_x.Time(1));
    nyquist_freq = Fs/2;  % Nyquist frequency
    Wn=cutoff_freq/nyquist_freq;    % non-dimensional frequency
    [filtb,filta]=butter(order,Wn,'low'); % construct the filter
    %%%%%%%%%%%%%%
    % filter inertial wind estimate
    wind_x_filt = filtfilt(filtb,filta,wind_x.Data);
    wind_y_filt = filtfilt(filtb,filta,wind_y.Data);
    wind_z_filt = filtfilt(filtb,filta,wind_z.Data);
    slip_filt = filtfilt(filtb,filta,slip);
    aoa_filt = filtfilt(filtb,filta,aoa);
    % filter vane wind estimate
    if (vane_measurements)
        slip_vane_filt = filtfilt(filtb,filta,slip_meas.Data-slip_bias_calib);
        aoa_vane_filt = filtfilt(filtb,filta,aoa_meas.Data-aoa_bias_calib);
        wind_x_vane_filt = filtfilt(filtb,filta,wind_x_vane.Data);
        wind_y_vane_filt = filtfilt(filtb,filta,wind_y_vane.Data);
        wind_z_vane_filt = filtfilt(filtb,filta,wind_z_vane.Data);
    end
    
    % comppute total magnitude of the wind
    wind_magnitude = sqrt(wind_x_filt.^2 + wind_y_filt.^2);
    wind_heading = 180/pi*(atan2(wind_x_filt,wind_y_filt));
    
    
    % standard wind plots
    fig0 = figure();
    fig0.Name = 'Estimated Wind';
    plot1 = subplot(2,1,1);
    %plot(time_resampled, wind_magnitude);
    hold on;
    plot(time_resampled, wind_x_filt);
    plot(time_resampled, wind_y_filt);
    xlabel('time (s)');
    ylabel('wind speed (m/s)');
    legend('wind east','wind north');
    grid on;
    plot2 = subplot(2,1,2);
    plot(time_resampled, wind_heading);
    xlabel('time (s)');
    ylabel('wind heading (deg)');
    ylim([-180,180]);
    grid on;


    % ENU wind plots
    fig1 = figure();
    fig1.Name = 'Estimated Wind';
    plot1 = subplot(3,1,1);
    hold on;
    plot(time_resampled, wind_x_filt);
    if vane_measurements
       plot(time_resampled, wind_x_vane_filt);
       legend('dynamic filtered','vanes');
    else
        legend('dynamic filtered');
    end
    xlabel('time (s)');
    ylabel('wind speed east (m/s)');
    grid on;
    
    plot2 = subplot(3,1,2);
    hold on;
    plot(time_resampled, wind_y_filt);
    if vane_measurements
       plot(time_resampled, wind_y_vane_filt);
       legend('dynamic filtered','vanes');
    else
        legend('dynamic filtered');
    end
    xlabel('time (s)');
    ylabel('wind speed north (m/s)');
    grid on;

    plot3_ = subplot(3,1,3);
    hold on;
    plot(time_resampled, wind_z_filt);
    if vane_measurements
       plot(time_resampled, wind_z_vane_filt);
       legend('dynamic filtered','vanes');
    else
        legend('dynamic filtered');
    end  
    xlabel('time (s)');
    ylabel('wind speed vertical (m/s)');
    grid on;
    
    % link the time axis
    linkaxes([plot1,plot2,plot3_],'x');
    hold off;
    
    % airflow plots
    fig6 = figure();
    fig6.Name = 'Airflow Angles';
    plot2 = subplot(2,1,2);
    hold on;
    plot(time_resampled, slip_filt);
    if vane_measurements
        plot(time_resampled, slip_vane_filt);
        legend('dynamic filtered', 'vane');
    else
        legend('filtered');
    end
    xlabel('time (s)');
    ylabel('AoS (rad)');
    grid on;
    ylim([-0.25,0.25]);
    a = fill([690,690,750,750], [-0.25,0.25,0.25,-0.25], 'r','Edgecolor','none','HandleVisibility','off');
    a.FaceAlpha = 0.1;
    
    plot1 = subplot(2,1,1);
    hold on;
    plot(time_resampled, aoa_filt+aoa_offset);
    if vane_measurements
        plot(time_resampled, aoa_vane_filt+aoa_offset);
        legend('dynamic filtered', 'vane');
    else
        legend('filtered');
    end
    xlabel('time (s)');
    ylabel('AoA (rad)');
    grid on;
    ylim([-0.1,0.25]);
    a = fill([690,690,750,750], [-0.25,0.25,0.25,-0.25], 'r','Edgecolor','none','HandleVisibility','off');
    a.FaceAlpha = 0.1;
    
    % link the time axis
    linkaxes([plot1,plot2],'x');
    hold off;

    
    % plot the difference in airspeed and inertial speed
    fig2 = figure();
    fig2.Name = 'airspeed scaling correctness';
    inertial_speed = sqrt(vel_0.Data.^2 + vel_1.Data.^2 + 0*vel_2.Data.^2);
    hold on;
    plot(time_resampled, inertial_speed);
    plot(time_resampled, aspd_scale_calib*TAS.Data);
    plot(time_resampled, aspd_scale_calib*CAS.Data);
    %plot(time_resampled, IAS.Data);
    legend('GPS', 'TAS scaled', 'CAS scaled');
    grid on;
    hold off;
    
    % 3D quiver plots of wind
    n = 2;
    fig3 = figure();
    fig3.Name = 'shear plot';
    plot3(pos_1.Data(1 : end),pos_0.Data(1 : end),-pos_2.Data(1 : end),'k.');
    axis equal
    hold all
    di = numel(pos_0.Data);
    distep = floor(di/20);

    quiver3(pos_1.Data(1 : n : end), pos_0.Data(1 : n : end), -pos_2.Data(1 : n : end),...
            wind_x_filt(1 : n : end), wind_y_filt(1 : n : end), wind_z_filt(1 : n : end), 0, 'r');
%     quiver3(pos_1.Data(2 : n : end), pos_0.Data(2 : n : end), -pos_2.Data(2 : n : end),...
%             wind_x_online.Data(2 : n : end), wind_y_online.Data(2 : n : end), wind_z_online.Data(2 : n : end), 0, 'b');
       
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    
    % export the wind data to be processed in python
    height = -1*pos_2.Data;
    home_dir = '\\wsl$\ubuntu-20.04\home\marvin\Master_Thesis_ADS\Git_Python\master-thesis-ads\wind_estimation\wind_data\';
    save(append(home_dir,'time.mat'),'time_resampled');
    save(append(home_dir,'height.mat'),'height');
    save(append(home_dir,'wind_x.mat'),'wind_x_filt');
    save(append(home_dir,'wind_y.mat'),'wind_y_filt');
    save(append(home_dir,'wind_z.mat'),'wind_z_filt');

        
        
end

