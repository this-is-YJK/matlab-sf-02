function sensorConfigs = sensor_config()
% SENSOR_CONFIG Define sensor configurations for platforms
% Returns an array of sensor configurations that can be mounted on platforms.
%
% Each sensor has:
%   - id: Unique sensor identifier
%   - name: Sensor name string
%   - type: 'radar', 'esm', 'eo', 'ir'
%   - hostPlatformId: ID of platform carrying this sensor
%   - mountPosition: [x; y; z] offset from platform centroid (body frame, meters)
%   - mountOrientation: [roll; pitch; yaw] offset from platform body frame (degrees)
%   - fov: Field of view parameters (sensor-type dependent)
%   - range: Maximum detection range (meters)
%   - updateRate: Measurement rate (Hz)
%   - noiseParams: Measurement noise parameters
%
% Returns:
%   sensorConfigs - Array of sensor configuration structs

%% Define sensor structure template
s = struct(...
    'id', 0, ...
    'name', '', ...
    'type', '', ...
    'hostPlatformId', 0, ...
    'mountPosition', [0; 0; 0], ...
    'mountOrientation', [0; 0; 0], ...
    'fov', struct(), ...
    'range', 0, ...
    'updateRate', 1, ...
    'noiseParams', struct(), ...
    'enabled', true);

% Pre-allocate for 4 sensors
sensorConfigs = repmat(s, 1, 4);

%% Sensor 1: Main surveillance radar on ownship (F_0)
sensorConfigs(1).id = 0;
sensorConfigs(1).name = "RADAR_01";
sensorConfigs(1).type = 'radar';
sensorConfigs(1).hostPlatformId = 0;  % Mounted on F_0
sensorConfigs(1).mountPosition = [0; 0; -2];  % 2m below centroid (nose-mounted)
sensorConfigs(1).mountOrientation = [0; 0; 0];  % Aligned with aircraft body

% Radar FOV: azimuth, elevation, range (conical scan)
sensorConfigs(1).fov.azimuth = 120;    % Total azimuth FOV in degrees (+/- 60)
sensorConfigs(1).fov.elevation = 40;   % Total elevation FOV in degrees (+/- 20)
sensorConfigs(1).fov.scanType = 'sector';  % 'sector', 'circular', 'fixed'

sensorConfigs(1).range = 200000;  % 200 km max range

% Radar measurement noise (1-sigma)
sensorConfigs(1).noiseParams.rangeStd = 50;       % meters
sensorConfigs(1).noiseParams.azimuthStd = 0.5;    % degrees
sensorConfigs(1).noiseParams.elevationStd = 0.5;  % degrees
sensorConfigs(1).noiseParams.rangeRateStd = 10;  % m/s (Doppler)

sensorConfigs(1).updateRate = 2;  % 2 Hz update
sensorConfigs(1).enabled = true;

%% Sensor 2: ESM (Electronic Support Measures) on ownship (F_0)
sensorConfigs(2).id = 1;
sensorConfigs(2).name = "ESM_01";
sensorConfigs(2).type = 'esm';
sensorConfigs(2).hostPlatformId = 0;  % Mounted on F_0
sensorConfigs(2).mountPosition = [0; 0; 1];  % 1m above centroid (top mounted)
sensorConfigs(2).mountOrientation = [0; 0; 0];

% ESM FOV: 360 degree coverage in azimuth, limited elevation
sensorConfigs(2).fov.azimuth = 360;    % Full 360 coverage
sensorConfigs(2).fov.elevation = 60;   % +/- 30 degree elevation
sensorConfigs(2).fov.scanType = 'circular';

sensorConfigs(2).range = 300000;  % 300 km (RF detection range)

% ESM measurement noise (AOA only - passively detects emitters)
sensorConfigs(2).noiseParams.azimuthStd = 2.0;    % degrees
sensorConfigs(2).noiseParams.elevationStd = 3.0;  % degrees

sensorConfigs(2).updateRate = 1;  % 1 Hz
sensorConfigs(2).enabled = true;

%% Sensor 3: Fire control radar on ownship (F_0) - narrow beam
sensorConfigs(3).id = 2;
sensorConfigs(3).name = "FCR_01";
sensorConfigs(3).type = 'radar';
sensorConfigs(3).hostPlatformId = 0;
sensorConfigs(3).mountPosition = [1; 0; 0];  % 1m forward of centroid
sensorConfigs(3).mountOrientation = [0; 0; 0];

% FCR has narrow FOV for target tracking
sensorConfigs(3).fov.azimuth = 20;      % Narrow azimuth (+/- 10 deg)
sensorConfigs(3).fov.elevation = 20;   % Narrow elevation (+/- 10 deg)
sensorConfigs(3).fov.scanType = 'fixed';

sensorConfigs(3).range = 100000;  % 100 km for fire control

% High precision for fire control
sensorConfigs(3).noiseParams.rangeStd = 10;
sensorConfigs(3).noiseParams.azimuthStd = 0.1;
sensorConfigs(3).noiseParams.elevationStd = 0.1;
sensorConfigs(3).noiseParams.rangeRateStd = 2;

sensorConfigs(3).updateRate = 10;  % 10 Hz for tracking
sensorConfigs(3).enabled = true;

%% Sensor 4: IRST (Infrared Search and Track) on ownship (F_0)
sensorConfigs(4).id = 3;
sensorConfigs(4).name = "IRST_01";
sensorConfigs(4).type = 'ir';
sensorConfigs(4).hostPlatformId = 0;
sensorConfigs(4).mountPosition = [0; 0; 0.5];  % Mounted on top
sensorConfigs(4).mountOrientation = [0; -15; 0];  % Canted down 15 degrees

% IRST has wide FOV, passive detection
sensorConfigs(4).fov.azimuth = 90;   % +/- 45 deg
sensorConfigs(4).fov.elevation = 30; % +/- 15 deg
sensorConfigs(4).fov.scanType = 'sector';

sensorConfigs(4).range = 50000;  % 50 km for IR detection (depends on target signature)

% IR measurement noise (angle only, passive)
sensorConfigs(4).noiseParams.azimuthStd = 1.0;
sensorConfigs(4).noiseParams.elevationStd = 1.5;
sensorConfigs(4).noiseParams.rangeEstimate = true;  % Can provide rough range via mono-pulse

sensorConfigs(4).updateRate = 5;  % 5 Hz
sensorConfigs(4).enabled = true;

end
