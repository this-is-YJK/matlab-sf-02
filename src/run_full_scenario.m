function run_full_scenario(config, varargin)
% RUN_FULL_SCENARIO Run complete simulation: trajectory + sensors
% Generates platform trajectories and processes sensor detections.
%
% Usage:
%   run_full_scenario(config)
%   run_full_scenario(config, 'PlotTrajectories', true)
%   run_full_scenario(config, 'RunSensors', true, 'PlotDetections', true)
%
% Inputs:
%   config  - Scenario configuration struct
%   varargin - Name-value pairs:
%       'PlotTrajectories' - Plot trajectory results (default: true)
%       'RunSensors'       - Run sensor simulation (default: true)
%       'PlotDetections'   - Plot sensor results (default: false)
%       'OutputDir'        - Output directory (default: 'output')
%
% This script:
%   1. Runs platform trajectory simulation (generates .scenario file)
%   2. Runs sensor simulation (generates .dat files per sensor)

%% Parse arguments
p = inputParser;
addRequired(p, 'config');
addParameter(p, 'PlotTrajectories', true, @islogical);
addParameter(p, 'RunSensors', true, @islogical);
addParameter(p, 'PlotDetections', false, @islogical);
addParameter(p, 'OutputDir', 'output', @ischar);
parse(p, config, varargin{:});

plotTraj = p.Results.PlotTrajectories;
runSensors = p.Results.RunSensors;
plotDet = p.Results.PlotDetections;
outputDir = p.Results.OutputDir;

%% Create output directory
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

%% Step 1: Run trajectory simulation
fprintf('=== Step 1: Platform Trajectory Simulation ===\n\n');

% Update config output
config.output.enabled = true;
config.output.filename = fullfile(outputDir, 'trajectories.scenario');

% Create platforms
platforms = create_platforms(config);

% Create data recorder
recorderObj = data_recorder(config);
platformConfigs = config.platforms;

% Time parameters
updateRate = config.scenario.updateRate;
stopTime = config.scenario.stopTime;
dt = 1.0 / updateRate;
numSamples = stopTime * updateRate + 1;

fprintf('Simulating %d platforms for %.0f seconds at %.1f Hz\n', ...
    size(platforms, 2), stopTime, updateRate);
fprintf('Output: %s\n\n', config.output.filename);

% Run simulation
for sampleNo = 1:numSamples
    simTime = (sampleNo - 1) * dt;

    % Advance each platform
    for platformIdx = 1:size(platforms, 2)
        platforms(platformIdx) = platforms(platformIdx).advance(dt);

        % Record data
        recorderObj.record(platforms(platformIdx), platformConfigs(platformIdx), ...
                          sampleNo, simTime);
    end

    % Progress update
    if mod(sampleNo, floor(numSamples / 10)) == 0
        fprintf('Progress: %d/%d samples (%.0f%%)\n', ...
            sampleNo, numSamples, 100 * sampleNo / numSamples);
    end
end

% Close recorder
recorderObj.close();

fprintf('\nTrajectory simulation complete!\n\n');

%% Step 2: Plot trajectories
if plotTraj
    fprintf('Plotting trajectories...\n');
    plot_trajectories(config);

    % Also create a 3D view
    figure('Name', '3D Trajectories', 'Color', 'w');
    plotTrajectories3D(recorderObj.data, platformConfigs);
end

%% Step 3: Run sensor simulation
if runSensors
    fprintf('\n=== Step 2: Sensor Simulation ===\n\n');

    sensorOutputDir = fullfile(outputDir, 'sensors');

    run_sensor_scenario(config.output.filename, ...
        'OutputDir', sensorOutputDir, ...
        'Verbose', true, ...
        'PlotResults', plotDet);
end

fprintf('\n=== Simulation Complete ===\n\n');
fprintf('Outputs written to: %s\n', outputDir);

end

function plotTrajectories3D(data, platformConfigs)
% PLOTTRAJECTORIES3D Create 3D trajectory plot

hold on;

colors = lines(length(platformConfigs));

for i = 1:length(platformConfigs)
    if i <= length(data) && ~isempty(data{i})
        d = data{i};

        % Extract position (lat, lon, alt)
        lat = d(:, 4);
        lon = d(:, 5);
        alt = d(:, 6) / 1000;  % Convert to km for display

        % Plot with color based on type
        if strcmpi(platformConfigs(i).type, 'ownship')
            plot3(lon, lat, alt, 'b-', 'LineWidth', 2);
        else
            plot3(lon, lat, alt, 'r--', 'LineWidth', 1.5);
        end

        % Mark start and end
        plot3(lon(1), lat(1), alt(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
        plot3(lon(end), lat(end), alt(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

        % Label
        text(lon(1), lat(1), alt(1) + 1, platformConfigs(i).name);
    end
end

xlabel('Longitude (deg)');
ylabel('Latitude (deg)');
zlabel('Altitude (km)');
title('Platform Trajectories (3D)');
legend([cellfun(@(x) x.name, platformConfigs, 'UniformOutput', false)], 'Location', 'best');
grid on;
view(30, 20);
axis tight;
hold off;
end
