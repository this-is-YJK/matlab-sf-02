%% example_custom_scenario.m
% Demonstrates how to use the modular scenario system with:
%   - Multiple ownships and targets
%   - LLA (geodetic) and ECEF (cartesian) coordinate inputs
%   - Custom output settings
%
% Run this script to execute the scenario.

clc; clear; close all;

%% Get base configuration
config = scenario_config();

%% Customize: Add third target with ECEF coordinates
% Extend struct array
n = length(config.platforms);
config.platforms(n+1) = config.platforms(1);  % Copy template
config.platforms(n+1).id = 3;
config.platforms(n+1).name = "T_3";
config.platforms(n+1).type = 'target';
config.platforms(n+1).coordSystem = 'ecef';
config.platforms(n+1).waypoints = [...
    20000000  80000000  20000000;   % ECEF in meters
    40000000  40000000  20000000];
config.platforms(n+1).times = [0; 1000];

%% Customize: Add second ownship
n = length(config.platforms);
config.platforms(n+1) = config.platforms(1);  % Copy template
config.platforms(n+1).id = 4;
config.platforms(n+1).name = "F_1";
config.platforms(n+1).type = 'ownship';
config.platforms(n+1).coordSystem = 'lla';
config.platforms(n+1).waypoints = [...
    10.0   20.0   30000;
    15.0   25.0   40000];
config.platforms(n+1).times = [0; 1000];

%% Customize: Change output filename
config.output.filename = 'custom_scenario_output.scenario';

%% Run the scenario
run_scenario(config);
