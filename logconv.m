% This Matlab Script can be used to import the binary logged values of the
% PX4FMU into data that can be plotted and analyzed.
% A maximum of 6 instances per topics are supported while converting. For
% plotting most plots only display the first instance.
% The script assumes that the topics instances exist in increasing order
% starting from 0.
% NOTE:
% - parameter importing uses modified python scripts **post- pyulog release
%   v0.6.0 -- until the next release, source compliation from the commit 
%   commit 3bc4f93 on https://github.com/PX4/pyulog/ master branch must be
%   done (once) before this script will execute properly 
% TODO:
% - properly catch all cases of a missing topic
% - properly support multi instance topics

% Clear everything
clc;
clear;
close all;

% nice latex plotting
set(0,'defaulttextinterpreter','latex');

addpath(genpath('01_draw_functions'));
addpath(genpath('02_helper_functions'));
addpath(genpath('03_kmltoolbox_v2.6'));
addpath(genpath('04_log_files'));
addpath(genpath('05_csv_files'));
addpath(genpath('06_mat_files'));
addpath(genpath('07_kmz_files'));
addpath(genpath('08_analysis_tools'));

% ************************************************************************
% SETTINGS (modify necessary parameter)
% ************************************************************************

% The log file name, no file specifier required
fileName = 'log_70_2022-9-8-18-19-22';
%log_90_2022-9-21-15-44-36
%log_86_2022-9-21-11-57-56
%log_82_2022-9-16-09-53-08
%EZG5_20211011_Gotthardpass_flight01
%log_77_2022-9-9-12-15-30
%log_72_2022-9-9-10-20-56 
%log_70_2022-9-8-18-19-22
%log_66_2022-9-6-12-12-22
%log_67_2022-9-6-12-34-14
%log_69_2022-9-8-17-54-32
%log_55_2022-8-24-14-01-52
%log_57_2022-8-24-14-36-30 
%log_56_2022-8-24-14-16-44 
%log_43_2022-8-21-16-12-48; 
%log_33_2022-7-28-12-55-42; 
%12_23_53, 13_22_34
%log_17_2022-6-30-11-42-36
%log_17_2022-6-30-11-42-36
%log_26_2022-7-14-10-43-18
%log_27_2022-7-14-11-21-50

% The log file location, only required when converting a .ulg file. All
% .csv files are supposed to be in 05_csv_files, and all .mat files are
% supposed to be in 06_mat_files
fileLocation = '04_log_files';

% the source from which the data is imported
% 0: converting the ulog to csv files and then parsing the csv files
%    (required for the first run)
% 1: only parsing the pre-existing csv files
%    (requires the generated csv files)
% 2: import the data from the .mat file
%    (requires the generated .mat file)
% else: Defaults to 0
loadingMode = 1;

% Print information while converting/loading the log file in mode 0 or 1.
% Helpfull to identify field missmatchs.
loadingVerbose = true;

% indicates if the sysvector map and the topics struct should be saved
% after they are generated.
saveMatlabData = true;

% delete the csv file after a run of the script
deleteCSVFiles = false;

% delimiter for the path
%   '/' for ubuntu
%   '\' for windows
pathDelimiter = '\';

% indicates if the plots should be generated. If set to false the log file
% is only converted to the sysvector.
generatePlots = true;

% only plot the logged data from t_start to t_end. If one of them is set to
% NaN all the logged data is plotted [s].
t_start = 350;
t_end = 550;

% change topic names or add new topics in the setupTopics function.

% ************************************************************
% Plotting settings
% ************************************************************

% *********************
% Enable/Disable Plots
% *********************
plotvector.gpsPlots = false;
plotvector.sensorPlots = false;
plotvector.differentialPressurePlots = false;
plotvector.estimatorPlots = false;
plotvector.estimatorStatusPlots = false;
plotvector.globalPositionPlots = false;
plotvector.windPlots = true;
plotvector.controlPlots = true;
plotvector.telemRSSIPlots = false;
plotvector.rawSensorPlots = false;
plotvector.cpuLoadPlots = false;
plotvector.distanceSensorPlots = false;
plotvector.missionResultPlots = false;
plotvector.vehicleStatusFlags = false;
plotvector.magVsThrustPlots = false;
plotvector.powerPlots = false;
plotvector.BatMonPlots = false;
plotvector.iridiumsbdStatusPlots = false;
plotvector.mpptPlots = false;
plotvector.airflowAnglePlots = false;

% *********************
% Link the Figure Axis Settings
% *********************
% Link the axes of different figure
plotvector.linkAxes = false;

% *********************
% GPS Plot Settings
% *********************
% Color mode for the 3D gps plot
% 0: Colored by altitude
% 1: Colored by GPS horizontal velocity
% 2: Red
% 3: Colored by the GPS velocity
% 4: Colored by the filtered airspeed
% Default: Colored by altitude
plotvector.colorModeGPS = 0;


% *********************
% Global Position Estimate Plot Settings
% *********************
% Color mode for the 3D estimated global position plot
% 0: Colored by the control mode
% 1: Red
% 2: Colored by altitude, not in Google Earth
% 3: Colored by the estimated ground velocity, not in Google Earth
% 4: Colored by the filtered airspeed, not in Google Earth
plotvector.colorModeGlobalPosition = 0;

% Indicates if the GPS position should be plot as a reference
plotvector.plotGPSReference = true;

% Indicates if in Google Earth a projection of the path to the ground
% should be displayed.
plotvector.plotPathShadow = true;

% Indicates if GoogleEarth should be autostarted.
plotvector.autostartGoogleEarth = true;

% *********************
% Differential Pressure Plot Settings
% *********************
% Indicates if the differential pressure and airspeed measurements should
% be corrected by the tube pressure loss according to: https://goo.gl/2d3WGc
plotvector.doPressureCorrection = false;

% Diameter of the pitot tube
plotvector.pressureCorrectionD = 0.003; % [m]

% Length of the pitot tube
plotvector.pressureCorrectionL = 0.65; % [m]

% DP reading of the sensor (101 for SDP3X, 62 for SDP600)
plotvector.pressureCorrectionDPSensor = 59.3314; % [Pa]

% Massflow (4.79e-7 for SDP3X, 6.17e-7 for SDP600)
plotvector.pressureCorrectionMassflow = 4.79e-7; % [kg/s]

% *********************
% Wind Plot Settings
% *********************
% Resampling time interval for displaying the wind vector.
plotvector.dtWindPlot = 0.5;

% Indicates if the ground speed vector should be added to the wind vectors
% plot.
plotvector.plotGroundSpeedVector = false;

% Indicates if the air speed vector should be added to the wind vectors
% plot.
plotvector.plotAirSpeedVector = false;

% ************************************************************************
% SETTINGS end
% ************************************************************************

% ******************
% Import the data
% ******************

% conversion factors
fconv_timestamp=1E-6;    % [microseconds] to [seconds]
fconv_gpsalt=1E-3;       % [mm] to [m]
fconv_gpslatlong=1E-7;   % [gps_raw_position_unit] to [deg]

if loadingMode==2
    if exist([fileName '.mat'], 'file') == 2
        load([fileName '.mat']);
        if (numel(fieldnames(topics)) == 0) || (numel(fieldnames(sysvector)) == 0)
            error(['Sysvector and/or topics loaded from the .mat file are empty.' newline ...
                'Run script first with loadingMode=0 and saveMatlabData=true'])
        end
    else
        error(['Could not load the data as the file does not exist.' newline ...
            'Run script first with loadingMode=0 and saveMatlabData=true'])
    end
else
    [sysvector, topics, paramvector, params] = ...
        ImportPX4LogData(fileName, fileLocation, '05_csv_files', ...
                         '06_mat_files', loadingMode, pathDelimiter, ...
                         fconv_timestamp, loadingVerbose, saveMatlabData, ...
                         deleteCSVFiles);
end

% ******************
% Crop the data
% ******************

[sysvector, paramvector] = CropPX4LogData(sysvector, paramvector, t_start, t_end);

% ******************
% Print the data
% ******************

if generatePlots
    DisplayPX4LogData(sysvector, topics, paramvector, params, fileName, fconv_gpsalt, fconv_gpslatlong, plotvector)
end

% export soaring state for analysis
ExportSoaringStates(sysvector, topics);
