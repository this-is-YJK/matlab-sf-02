# MATLAB Tracking Scenario Simulator

A modular, extensible platform for simulating multi-platform tracking scenarios with geodetic and ECEF coordinate support.

## Overview

This simulator generates realistic platform trajectories for tracking and sensor fusion research. It supports multiple ownships and targets with configurable waypoints in either geodetic (LLA) or ECEF (XYZ) coordinates.

## Requirements

- MATLAB R2020a or later
- Sensor Fusion and Tracking Toolbox (for `trackingScenario`)
- Mapping Toolbox (for WGS84 ellipsoid functions)

## Quick Start

```matlab
% Run with default configuration
run senario_10062026_01.m

% Or run with custom configuration
config = scenario_config();
% ... modify config ...
run_scenario(config);
```

## File Structure

```
src/
  senario_10062026_01.m     - Main entry point (trajectories)
  scenario_config.m          - Platform configuration definition
  sensor_config.m            - Sensor configuration definition
  run_scenario.m             - Trajectory simulation orchestrator
  run_sensor_scenario.m      - Sensor simulation from .scenario file
  run_full_scenario.m        - Combined: trajectory + sensors
  create_platforms.m         - Platform creation
  waypoint_utils.m           - Coordinate conversions (LLA/ECEF)
  attitude_utils.m           - Attitude calculations (heading, pitch, roll)
  SensorBase.m               - Abstract base class for sensors
  RadarSensor.m              - Active radar sensor model
  ESMSensor.m                - ESM/RWR passive sensor model
  IRSensor.m                 - Infrared sensor model
  data_recorder.m            - Trajectory data capture
  detection_recorder.m       - Sensor detection capture
  plot_trajectories.m        - 3D trajectory visualization
  example_custom_scenario.m  - Platform configuration examples
  example_sensor_scenario.m  - Sensor configuration examples
docs/
  mathematics.html           - Mathematical reference (MathJax)
```

## Configuration

### Adding Platforms

```matlab
% Get base configuration
config = scenario_config();

% Extend with new platform
n = length(config.platforms);
config.platforms(n+1) = config.platforms(1);  % Copy template
config.platforms(n+1).id = 3;
config.platforms(n+1).name = "T_3";
config.platforms(n+1).type = 'target';
config.platforms(n+1).coordSystem = 'lla';  % or 'ecef'
config.platforms(n+1).waypoints = [lat, lon, alt];
config.platforms(n+1).times = [t0; tf];

run_scenario(config);
```

### Coordinate Systems

| System | Units | Format |
|--------|-------|--------|
| `lla` | degrees, degrees, meters | [latitude, longitude, altitude] |
| `ecef` | meters | [X, Y, Z] Earth-Centered Earth-Fixed |

### Platform Types

- `ownship` - Blue force platforms (plotted in blue)
- `target` - Red force platforms (plotted in red)

## Output Format

The simulator generates a CSV file with the following columns:

| Column | Description | Units |
|--------|-------------|-------|
| Sample No | Sequential sample number | - |
| Time | Simulation time | seconds |
| target_ID | Platform identifier | integer |
| Target_name | Platform name | string |
| Latitude | Geodetic latitude | degrees |
| Longitude | Geodetic longitude | degrees |
| Altitude | Height above WGS84 ellipsoid | meters |
| Velocity_x | ECEF X velocity component | m/s |
| Velocity_y | ECEF Y velocity component | m/s |
| Velocity_z | ECEF Z velocity component | m/s |
| Heading | Azimuth angle | degrees |
| Roll | Bank angle | degrees |
| Pitch | Elevation angle | degrees |
| Yaw | Same as heading | degrees |

## Sensor Simulation

The simulator includes comprehensive sensor models for multi-sensor fusion research.

### Supported Sensor Types

| Type | Measurements | Description |
|------|-------------|-------------|
| **Radar** | Range, Azimuth, Elevation, Range-rate | Active RF sensor with SNR-based detection |
| **ESM** | Azimuth, Elevation | Passive radar warning receiver (emitter detection) |
| **IR** | Azimuth, Elevation, Range estimate | Infrared Search and Track |

### Sensor Configuration

```matlab
% Get default sensor configuration
sensorConfigs = sensor_config();

% Each sensor has:
%   .id              - Unique sensor identifier
%   .name            - Sensor name
%   .type            - 'radar', 'esm', 'ir'
%   .hostPlatformId  - Platform ID to mount on
%   .mountPosition   - [x; y; z] offset from platform centroid (m)
%   .mountOrientation - [roll; pitch; yaw] offset from body frame (deg)
%   .fov             - Field of view: .azimuth, .elevation, .scanType
%   .range           - Maximum detection range (m)
%   .updateRate       - Measurement rate (Hz)
%   .noiseParams      - Measurement noise standard deviations
```

### Running Sensor Simulation

```matlab
% Method 1: From existing trajectory file
run_sensor_scenario('output/trajectory.scenario', ...
    'OutputDir', 'sensor_output');

% Method 2: Full simulation (trajectories + sensors)
config = scenario_config();
run_full_scenario(config, 'RunSensors', true);
```

### Sensor Output Format

Each sensor generates a `.dat` file with detection records:

**Radar columns:** Time, Target_ID, Name, Range_m, Az_deg, El_deg, RangeRate_mps, SNR_dB, Pd, RCS_m2, GT_*

**ESM columns:** Time, Target_ID, Name, Az_deg, El_deg, SignalStrength, EmitterClass, PRF_Hz, PulseWidth_us, GT_*

**IR columns:** Time, Target_ID, Name, Az_deg, El_deg, RangeEst_m, IRIntensity, Signature, GT_*

### Sensor FOV Model

```
        ^ Sensor X (boresight)
        |
        |   / \
        |  /   \  <- Azimuth FOV (+/- 60 deg)
        | /_____\
        |
        +----------> Sensor Y
       /
      v Sensor Z

Elevation FOV: +/- 20 deg (vertical spread)
```

### Mounting and Attitude

Sensor orientation = Platform attitude + Mount offset

- Platform attitude derived from velocity vector (heading, pitch, roll)
- Mount offset allows sensor to be canted relative to platform body frame
- Sensor-to-world transformation computed per time step

## Mathematics Reference

See `docs/mathematics.html` for detailed mathematical derivations of:

- Coordinate transformations (ECEF/LLA)
- Attitude calculation algorithms
- Sensor measurement equations
- Field of View calculations
- Radar detection model (SNR, Pd)

## Customization

### Modifying Update Rate and Duration

```matlab
config.scenario.updateRate = 10;   % 10 Hz
config.scenario.stopTime = 500;    % 500 seconds
```

### Custom Output Filename

```matlab
config.output.filename = 'my_scenario.scenario';
config.output.enabled = true;
```

### Adding Complex Trajectories

```matlab
% Multi-leg flight path with intermediate waypoints
config.platforms(1).waypoints = [
    35.0  45.0  10000;  % Start
    35.0  46.0  12000;  % Climb leg
    34.5  47.0  12000;  % Level turn
    34.0  48.0  10000]; % Descent
config.platforms(1).times = [0; 200; 400; 600];
```

## License

MIT License - Free for academic and research use.
