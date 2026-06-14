function config = scenario_config()
% SCENARIO_CONFIG Define scenario configuration
% Returns a configuration struct containing all scenario parameters,
% platform definitions, and output settings.
%
% Returns:
%   config - Struct with scenario, platforms, and output settings

%% Scenario Parameters
config.scenario.updateRate = 1;      % Hz
config.scenario.stopTime = 1000;      % seconds

%% Platforms Configuration
% Each platform is defined with:
%   - id: unique integer identifier
%   - name: string identifier
%   - type: 'ownship' or 'target'
%   - coordSystem: 'lla' (geodetic) or 'ecef' (cartesian)
%   - waypoints: [lat, lon, alt] for LLA or [x, y, z] for ECEF (meters)
%   - times: time vector for waypoints

% Define platform structure template
p = struct('id', 0, 'name', '', 'type', '', 'coordSystem', '', 'waypoints', [], 'times', []);

% Pre-allocate struct array for 3 platforms
config.platforms = repmat(p, 1, 3);

% Ownship (F_0)
config.platforms(1).id = 0;
config.platforms(1).name = "F_0";
config.platforms(1).type = 'ownship';
config.platforms(1).coordSystem = 'lla';
config.platforms(1).waypoints = [...
    35.298   45      28270;   % start
    20.388   21.801  51070];  % end (lat, lon, alt in meters)
config.platforms(1).times = [0; 1000];

% Target 1 (T_1)
config.platforms(2).id = 1;
config.platforms(2).name = "T_1";
config.platforms(2).type = 'target';
config.platforms(2).coordSystem = 'lla';
config.platforms(2).waypoints = [...
    13.64    75.964  78476;
    19.484   45      53624];
config.platforms(2).times = [0; 1000];

% Target 2 (T_2)
config.platforms(3).id = 2;
config.platforms(3).name = "T_2";
config.platforms(3).type = 'target';
config.platforms(3).coordSystem = 'lla';
config.platforms(3).waypoints = [...
    12.61    63.435  85274;
    24.113   63.435  42615];
config.platforms(3).times = [0; 1000];

%% Output Settings
config.output.enabled = true;
config.output.filename = 'scenario_output.scenario';
config.output.header = ['Sample No, Time, target_ID, Target_name, ' ...
    'Latitude, Longitude, Altitude, ' ...
    'Velocity_x, Velocity_y, Velocity_z, ' ...
    'Heading, Roll, Pitch, Yaw'];

end
