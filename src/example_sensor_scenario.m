%% Example: Sensor Simulation from Existing Trajectory Data
% This example demonstrates how to run sensor simulations
% on recorded trajectory data from a .scenario file.
%
% Prerequisites:
%   1. Run trajectory simulation first (senario_10062026_01.m)
%   2. This creates 'trajectory.scenario' file
%   3. Sensor simulation reads this file and generates detections

%% Example 1: Run sensors on existing trajectory file

% Specify the trajectory file
scenarioFile = 'output/trajectory.scenario';

% Check if file exists
if exist(scenarioFile, 'file')
    % Run sensor simulation with default configuration
    run_sensor_scenario(scenarioFile, ...
        'OutputDir', 'output/sensor_data', ...
        'Verbose', true);
else
    fprintf('Scenario file not found: %s\n', scenarioFile);
    fprintf('Run trajectory simulation first.\n');
end

%% Example 2: Run complete simulation (trajectories + sensors)

% Get base scenario configuration
% config = scenario_config();

% Customize sensor config (optional)
% See sensor_config.m for all parameters

% Run full simulation
% run_full_scenario(config, ...
%     'PlotTrajectories', true, ...
%     'RunSensors', true, ...
%     'OutputDir', 'output/full_run');

%% Example 3: Custom sensor configuration

% Define custom sensors
function customSensors = custom_sensor_config()
    % Template
    s = struct('id', 0, 'name', '', 'type', '', ...
               'hostPlatformId', 0, ...
               'mountPosition', [0; 0; 0], ...
               'mountOrientation', [0; 0; 0], ...
               'fov', struct(), ...
               'range', 0, ...
               'updateRate', 1, ...
               'noiseParams', struct(), ...
               'enabled', true);

    % Single radar sensor
    customSensors = s;
    customSensors.id = 0;
    customSensors.name = "Custom_Radar";
    customSensors.type = 'radar';
    customSensors.hostPlatformId = 0;
    customSensors.mountPosition = [0; 0; -3];
    customSensors.mountOrientation = [0; 0; 0];
    customSensors.fov.azimuth = 60;
    customSensors.fov.elevation = 30;
    customSensors.range = 150000;
    customSensors.noiseParams.rangeStd = 30;
    customSensors.noiseParams.azimuthStd = 0.3;
    customSensors.noiseParams.elevationStd = 0.3;
    customSensors.noiseParams.rangeRateStd = 5;
    customSensors.updateRate = 4;
end

% Usage:
% run_sensor_scenario(scenarioFile, 'SensorConfig', @custom_sensor_config);

%% Example 4: Processing detection data

% After running sensor simulation, read .dat files
% data = load('output/sensor_data/RADAR_01_detections.dat');

% File format:
% Radar: Time, TargetID, Name, Range, Az, El, RangeRate, SNR, Pd, RCS, GT_*
% ESM:   Time, TargetID, Name, Az, El, SignalStrength, EmitterClass, PRF, PW, GT_*
% IR:    Time, TargetID, Name, Az, El, RangeEst, IRIntensity, Signature, GT_*

%% Notes on sensor types:
%
% RADAR: Active sensor providing range, angles, and range-rate
%        - Detection range depends on target RCS and radar power
%        - SNR model: decreases with R^4
%
% ESM: Passive sensor detecting radar emissions
%        - Provides angle-of-arrival only (no range)
%        - Only detects targets with active emitters
%
% IR: Passive thermal detector
%        - Angle measurements with optional range estimate
%        - Detection depends on thermal signature
%        - Affected by atmospheric conditions
