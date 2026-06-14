function run_sensor_scenario(scenarioFile, varargin)
% RUN_SENSOR_SCENARIO Run sensor simulation against recorded trajectory data
% Reads platform trajectory data from .scenario file and generates sensor detections.
%
% Usage:
%   run_sensor_scenario('trajectory.scenario')
%   run_sensor_scenario('trajectory.scenario', 'OutputDir', 'sensor_output')
%   run_sensor_scenario('trajectory.scenario', 'SensorConfig', @custom_sensor_config)
%
% Inputs:
%   scenarioFile - Path to .scenario file with trajectory data
%   varargin     - Name-value pairs:
%       'OutputDir'    - Output directory for .dat files (default: 'output')
%       'SensorConfig' - Function handle returning sensor configs
%       'Verbose'      - Print progress (default: true)
%       'PlotResults'  - Plot final results (default: false)
%
% Output:
%   Creates .dat files in output directory, one per sensor
%
% File Format (.scenario):
%   CSV with header row, columns:
%   Sample_No, Time, Target_ID, Target_Name, Lat, Lon, Alt, Vx, Vy, Vz, Heading, Roll, Pitch, Yaw

%% Parse input arguments
p = inputParser;
addRequired(p, 'scenarioFile', @ischar);
addParameter(p, 'OutputDir', 'output', @ischar);
addParameter(p, 'SensorConfig', @sensor_config, @(x) isa(x, 'function_handle'));
addParameter(p, 'Verbose', true, @islogical);
addParameter(p, 'PlotResults', false, @islogical);
parse(p, scenarioFile, varargin{:});

outputDir = p.Results.OutputDir;
sensorConfigFunc = p.Results.SensorConfig;
verbose = p.Results.Verbose;
plotResults = p.Results.PlotResults;

%% Check scenario file exists
if ~exist(scenarioFile, 'file')
    error('Scenario file not found: %s', scenarioFile);
end

%% Load trajectory data
if verbose
    fprintf('Loading trajectory data from: %s\n', scenarioFile);
end

trajectoryData = loadScenarioFile(scenarioFile);

if verbose
    fprintf('  Loaded %d records\n', size(trajectoryData, 1));
    fprintf('  Time range: %.1f to %.1f seconds\n', ...
        min(trajectoryData(:, 2)), max(trajectoryData(:, 2)));
end

%% Get unique platforms and times
platformIds = unique(trajectoryData(:, 3));
times = unique(trajectoryData(:, 2));
numPlatforms = length(platformIds);
numTimeSteps = length(times);

if verbose
    fprintf('  Platforms: %d\n', numPlatforms);
    fprintf('  Time steps: %d\n', numTimeSteps);
end

%% Build platform data structures
% platformData{i} = matrix of all data for platform with id = platformIds(i)
platformData = cell(numPlatforms, 1);
platformNames = cell(numPlatforms, 1);

for i = 1:numPlatforms
    platformId = platformIds(i);
    rows = trajectoryData(:, 3) == platformId;
    platformData{i} = trajectoryData(rows, :);

    % Get platform name (from first record)
    % Note: Name is string, stored differently - we'll handle this
    nameIdx = find(rows, 1, 'first');
    platformNames{i} = sprintf('Platform_%d', platformId);
end

%% Load sensor configurations
sensorConfigs = sensorConfigFunc();
numSensors = length(sensorConfigs);

if verbose
    fprintf('\nSensor configuration:\n');
    for i = 1:numSensors
        fprintf('  [%d] %s (%s) on Platform %d, Range: %.0f km\n', ...
            sensorConfigs(i).id, sensorConfigs(i).name, ...
            sensorConfigs(i).type, sensorConfigs(i).hostPlatformId, ...
            sensorConfigs(i).range / 1000);
    end
end

%% Create sensor objects
sensors = cell(numSensors, 1);
rng = RandStream('mlfg6331_64', 'Seed', 12345);

for i = 1:numSensors
    cfg = sensorConfigs(i);
    switch lower(cfg.type)
        case 'radar'
            sensors{i} = RadarSensor(cfg, rng);
        case 'esm'
            sensors{i} = ESMSensor(cfg, rng);
        case 'ir'
            sensors{i} = IRSensor(cfg, rng);
        otherwise
            sensors{i} = SensorBase(cfg, rng);
    end
end

%% Create detection recorder
recorder = detection_recorder(sensorConfigs, outputDir, true);

%% Run simulation
if verbose
    fprintf('\nRunning sensor simulation...\n');
end

detectionCount = zeros(numSensors, 1);
missCount = zeros(numSensors, 1);

for tIdx = 1:numTimeSteps
    currentTime = times(tIdx);

    % Get all platform states at this time
    platformStates = struct();
    platformPoses = struct();

    for pIdx = 1:numPlatforms
        platformId = platformIds(pIdx);
        pData = platformData{pIdx};

        % Find record closest to current time
        [~, timeRowIdx] = min(abs(pData(:, 2) - currentTime));
        row = pData(timeRowIdx, :);

        % Parse data (adjust indices based on your .scenario format)
        % Expected columns: Sample, Time, ID, (Name), Lat, Lon, Alt, Vx, Vy, Vz, Heading, Roll, Pitch, Yaw
        pose = struct();
        pose.Position = lla2ecef(wgs84Ellipsoid('kilometer'), row(5), row(6), row(7));
        pose.Velocity = [row(8), row(9), row(10)]';

        attitude = struct();
        attitude.heading = row(11);
        attitude.pitch = row(13);
        attitude.roll = row(12);

        % Fix position array shape
        pose.Position = pose.Position(:)';

        platformStates(platformId).pose = pose;
        platformStates(platformId).attitude = attitude;
        platformStates(platformId).name = platformNames{pIdx};
        platformStates(platformId).type = 'aircraft';  % Default
    end

    % Process each sensor
    for sIdx = 1:numSensors
        sensor = sensors{sIdx};
        cfg = sensorConfigs(sIdx);

        % Check if sensor should update at this time
        if ~sensor.shouldUpdate(currentTime)
            continue;
        end

        % Get host platform state
        hostId = cfg.hostPlatformId;
        if ~isfield(platformStates, num2str(hostId)) && ...
           ~any(platformIds == hostId)
            continue;
        end

        % Get host platform index
        hostIdx = find(platformIds == hostId, 1);
        if isempty(hostIdx)
            continue;
        end

        hostState = platformStates(num2str(hostId));

        % Compute sensor pose in world frame
        sensorPose = sensor.getSensorPoseInWorld(hostState.pose, hostState.attitude);

        % Detect each target platform
        for pIdx = 1:numPlatforms
            targetId = platformIds(pIdx);

            % Skip own platform
            if targetId == hostId
                continue;
            end

            targetState = platformStates(num2str(targetId));

            % Generate detection
            switch lower(cfg.type)
                case 'radar'
                    detection = sensor.detectRadar(...
                        targetState.pose, ...
                        targetState.type, ...
                        sensorPose, ...
                        sensorPose.attitude, ...
                        currentTime);

                case 'esm'
                    % ESM requires emitter - assume all have active radar
                    targetHasEmitter = true;
                    detection = sensor.detectESM(...
                        targetState.pose, ...
                        targetState.type, ...
                        sensorPose, ...
                        sensorPose.attitude, ...
                        currentTime, ...
                        targetHasEmitter);

                case 'ir'
                    detection = sensor.detectIR(...
                        targetState.pose, ...
                        targetState.type, ...
                        sensorPose, ...
                        sensorPose.attitude, ...
                        currentTime);

                otherwise
                    detection = sensor.detect(...
                        targetState.pose, ...
                        targetState.type, ...
                        sensorPose, ...
                        sensorPose.attitude, ...
                        currentTime);
            end

            % Record detection
            recorder.recordDetection(cfg.id, detection, targetId, targetState.name);

            if detection.valid
                detectionCount(sIdx) = detectionCount(sIdx) + 1;
            else
                missCount(sIdx) = missCount(sIdx) + 1;
            end
        end
    end

    % Progress update
    if verbose && mod(tIdx, max(1, floor(numTimeSteps / 10))) == 0
        fprintf('  Progress: %d/%d time steps (%.1f%%)\n', ...
            tIdx, numTimeSteps, 100 * tIdx / numTimeSteps);
    end
end

%% Close recorder and print summary
recorder.close();

if verbose
    fprintf('\nSimulation complete!\n');
    fprintf('\nDetection Summary:\n');
    fprintf('%-15s %-10s %-10s %-10s\n', 'Sensor', 'Detections', 'Misses', 'P Detect');
    fprintf('%-15s %-10s %-10s %-10s\n', '---------------', '----------', '----------', '----------');
    for sIdx = 1:numSensors
        total = detectionCount(sIdx) + missCount(sIdx);
        if total > 0
            pDetect = detectionCount(sIdx) / total;
        else
            pDetect = 0;
        end
        fprintf('%-15s %-10d %-10d %-10.2f\n', ...
            sensorConfigs(sIdx).name, detectionCount(sIdx), missCount(sIdx), pDetect);
    end

    fprintf('\nOutput files written to: %s\n', outputDir);
end

%% Optional: Plot results
if plotResults
    plotSensorGeometry(platformStates, platformIds, platformNames, sensorConfigs);
end

end

%% Helper Functions

function data = loadScenarioFile(filename)
% LOADSCENARIOFILE Load trajectory data from .scenario file
% Handles both CSV with header and headerless formats

fid = fopen(filename, 'r');
if fid < 0
    error('Could not open file: %s', filename);
end

% Read all lines
lines = {};
while ~feof(fid)
    line = fgetl(fid);
    if ischar(line) && ~isempty(line)
        lines{end+1} = line;
    end
end
fclose(fid);

% Find header (first non-comment line starting with letters)
headerLine = 1;
dataStart = 1;
for i = 1:length(lines)
    line = lines{i};
    if startsWith(strtrim(line), '%')
        continue;  % Comment line
    end
    firstChar = strtrim(line(1:min(10, length(line))));
    if ~isempty(regexp(firstChar, '^[A-Za-z]', 'once'))
        headerLine = i;
        dataStart = i + 1;
        break;
    elseif ~isempty(regexp(firstChar, '^\d', 'once'))
        dataStart = i;
        break;
    end
end

% Parse data rows
numDataRows = length(lines) - dataStart + 1;
maxCols = 14;  % Expected columns

data = zeros(numDataRows, maxCols);
rowIdx = 0;

for i = dataStart:length(lines)
    line = lines{i};
    if isempty(strtrim(line)) || startsWith(strtrim(line), '%')
        continue;
    end

    % Parse CSV
    parts = strsplit(line, ',');
    numParts = min(length(parts), maxCols);

    rowIdx = rowIdx + 1;
    for j = 1:numParts
        val = strtrim(parts{j});
        if ~isempty(regexp(val, '^-?\d', 'once'))
            data(rowIdx, j) = str2double(val);
        end
    end
end

% Trim to actual size
data = data(1:rowIdx, :);
end

function plotSensorGeometry(platformStates, platformIds, platformNames, sensorConfigs)
% PLOTSENSORGEOMETRY Visualize sensor coverage

figure('Name', 'Sensor Geometry', 'Color', 'w');

hold on;

% Plot platforms
for i = 1:length(platformIds)
    state = platformStates(num2str(platformIds(i)));
    pos = state.pose.Position;

    % Normalize for visualization
    plot3(pos(1), pos(2), pos(3), 'o', 'MarkerSize', 10, 'LineWidth', 2);
    text(pos(1), pos(2), pos(3) + 5000, platformNames{i}, 'FontSize', 8);
end

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Platform Positions and Sensor Geometry');

grid on;
axis equal;
view(30, 20);
hold off;
end
