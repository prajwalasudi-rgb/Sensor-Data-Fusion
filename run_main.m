%RUN_MAIN  Entry point for the Master Thesis Kalman-filter fusion simulator.
%
%   Open MATLAB, cd into this repository folder and type:  run_main
%
%   By default it runs the final 2D/3D sensor-fusion simulator
%   (src/main/New_filter.m). Set EXPERIMENT below to run one of the
%   parameter studies instead. Figures are written to ./results/.
%
%   Required MATLAB toolboxes:
%     - Control System Toolbox        (ss, lsim)
%     - Statistics and Machine Learning Toolbox (normrnd)

clear; clc; close all;

repoRoot = fileparts(mfilename('fullpath'));
addpath(fullfile(repoRoot, 'src', 'main'));
addpath(fullfile(repoRoot, 'src', 'experiments'));
addpath(fullfile(repoRoot, 'src', 'utils'));

EXPERIMENT = 'main';   % 'main' | 'position_noise' | 'velocity_noise' | 'RQ_tuning' | 'initial_P'

switch EXPERIMENT
    case 'main'            % final fusion filter -> results/main_fusion
        New_filter;
    case 'position_noise'  % sweep of 2D/3D position-sensor noise -> results/Xerror, Yerror, Therror, rms
        F_error;
    case 'velocity_noise'  % sweep of velocity-sensor noise -> results/Xerror_v, Yerror_v, Therror_v, rms
        F_errorV;
    case 'RQ_tuning'       % effect of R and Q on error variance / Kalman gain -> results/Ini_P
        F_RQ;
    case 'initial_P'       % effect of initial covariance P0 (plots only, saving disabled)
        F_iniP;
    otherwise
        error('Unknown EXPERIMENT "%s"', EXPERIMENT);
end
