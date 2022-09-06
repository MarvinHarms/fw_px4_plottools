% Clear everything
clc;
clear;
close all;

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
fileName = 'log_57_2022-8-24-14-36-30';

t_start = 150;%350
t_end = 500;%430
