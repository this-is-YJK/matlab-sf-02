function run_scenario(config)
% RUN_SCENARIO Execute the tracking scenario simulation
% Main entry point for running a scenario simulation.
%
% Input:
%   config - Configuration struct (optional, defaults to scenario_config())
%
% Example:
%   config = scenario_config();
%   run_scenario(config);
%
%   % Or with custom configuration:
%   config.platforms(4) = struct('id', 3, 'name', "T_3", ...
%       'type', 'target', 'coordSystem', 'ecef', ...
%       'waypoints', [20000 80000 20000; 40000 40000 20000], ...
%       'times', [0; 1000]);
%   run_scenario(config);

if nargin < 1
    config = scenario_config();
end

%% Create Scenario
scenario = trackingScenario(...
    'UpdateRate', config.scenario.updateRate, ...
    'StopTime', config.scenario.stopTime);

%% Create Platforms
platforms = create_platforms(scenario, config);

%% Initialize Data Recorder
recorder = data_recorder(config);

%% Simulation Loop
sampleNo = 1;
disp('Starting simulation...');

while advance(scenario)
    for i = 1:length(platforms)
        recorder.record(platforms{i}, config.platforms(i), ...
            sampleNo, scenario.SimulationTime);
        sampleNo = sampleNo + 1;
    end
end

recorder.close();
disp(['Simulation complete. Output saved to: ' config.output.filename]);

%% Plot Results
plot_trajectories(recorder, config);

end
