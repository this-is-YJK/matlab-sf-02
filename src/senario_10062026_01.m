%% main.m - Scenario Simulation Entry Point
% Run this script to execute the default scenario.
%
% For customization, see:
%   - scenario_config.m       : Modify platform definitions
%   - example_custom_scenario.m: Example with multiple platforms and coord systems
%
% Architecture:
%   1. scenario_config()     - Define all scenario parameters
%   2. run_scenario()         - Execute simulation
%      └── create_platforms() - Create platform objects
%      └── data_recorder      - Record and export trajectory data
%   3. plot_trajectories()    - Visualize results

clc; clear; close all;

%% Run with default configuration
config = scenario_config();
run_scenario(config);
