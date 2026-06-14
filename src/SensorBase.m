classdef SensorBase < handle
% SENSORBASE Abstract base class for all sensor types
% Provides common functionality for:
%   - Field of View (FOV) checks
%   - Sensor mounting and coordinate transformations
%   - Measurement noise modeling
%   - Platform attitude integration
%
% Subclasses must implement:
%   - generateMeasurement() - Generate sensor-specific measurements
%
% Properties:
%   config     - Sensor configuration struct
%   lastMeasurement - Most recent measurement
%   measurementHistory - All measurements since reset
%   timeOfLastUpdate - Time of last update

properties
    config
    lastMeasurement
    measurementHistory
    timeOfLastUpdate
    rng  % Random number generator stream
end

properties (Access = protected)
    wgs  % WGS84 ellipsoid
end

methods
    function obj = SensorBase(config, rngStream)
        % SENSORBASE Constructor
        % Inputs:
        %   config    - Sensor configuration struct
        %   rngStream - RandStream object (optional, defaults to global)

        obj.config = config;
        obj.timeOfLastUpdate = 0;
        obj.lastMeasurement = [];
        obj.measurementHistory = {};
        obj.wgs = wgs84Ellipsoid('kilometer');

        if nargin < 2 || isempty(rngStream)
            obj.rng = RandStream('mlfg6331_64', 'Seed', 0);
        else
            obj.rng = rngStream;
        end
    end

    function reset(obj)
        % RESET Clear measurement history
        obj.lastMeasurement = [];
        obj.measurementHistory = {};
        obj.timeOfLastUpdate = 0;
    end

    function isInFOV = checkFOV(obj, relativePos, sensorAttitude)
        % CHECKFOV Check if target is within sensor field of view
        % Inputs:
        %   relativePos - Target position relative to sensor in sensor frame [x; y; z] meters
        %   sensorAttitude - Sensor attitude struct with .heading, .pitch, .roll
        % Returns:
        %   isInFOV - Boolean indicating if target is visible

        % Calculate angles to target in sensor frame
        range = norm(relativePos);

        if range < 1  % Too close
            isInFOV = false;
            return;
        end

        % Azimuth angle in sensor frame
        azimuth = atan2d(relativePos(2), relativePos(1));

        % Elevation angle in sensor frame
        horizontalDist = sqrt(relativePos(1)^2 + relativePos(2)^2);
        elevation = atan2d(relativePos(3), horizontalDist);

        % Check against FOV limits
        halfAz = obj.config.fov.azimuth / 2;
        halfEl = obj.config.fov.elevation / 2;

        isInFOV = (abs(azimuth) <= halfAz) && (abs(elevation) <= halfEl);
    end

    function [az, el, r] = cartesianToSpherical(obj, cartesian)
        % CARTESIANTOSPHERICAL Convert Cartesian to spherical coordinates
        % Input:
        %   cartesian - [x; y; z] in meters
        % Returns:
        %   az - Azimuth angle in degrees (from +x, towards +y)
        %   el - Elevation angle in degrees (from horizontal)
        %   r  - Range in meters

        x = cartesian(1);
        y = cartesian(2);
        z = cartesian(3);

        r = sqrt(x^2 + y^2 + z^2);

        if r < 1e-6
            az = 0;
            el = 0;
            return;
        end

        az = atan2d(y, x);
        horizontalDist = sqrt(x^2 + y^2);
        el = atan2d(z, horizontalDist);
    end

    function sensorPose = getSensorPoseInWorld(obj, platformPose, platformAttitude)
        % GETSENSORPOSEINWORLD Compute sensor position and orientation in world frame
        % Inputs:
        %   platformPose      - Platform pose struct with .Position (ECEF)
        %   platformAttitude  - Platform attitude struct with .heading, .pitch, .roll
        % Returns:
        %   sensorPose - Struct with .position and .orientation (DCM)

        % Convert platform attitude to DCM (NED to body)
        R_body_from_ned = attitudeToDCM(platformAttitude.heading, ...
                                        platformAttitude.pitch, ...
                                        platformAttitude.roll);

        % Mount position offset in body frame
        mountPos_body = obj.config.mountPosition;

        % Transform mount offset to ECEF (approximate, assumes local tangent plane)
        % Create local NED to ECEF rotation matrix
        [lat, ~] = ecef2geodetic(obj.wgs, ...
            platformPose.Position(1), platformPose.Position(2), platformPose.Position(3));

        R_ned_from_ecef = nedRotationMatrix(lat, platformPose.Position);

        % Mount position in ECEF
        mountOffset_body = R_body_from_ned * mountPos_body;
        mountOffset_ned = mountOffset_body;  % Same in NED orientation sense
        mountOffset_ecef = (R_ned_from_ecef') * mountOffset_ned;

        sensorPosition = platformPose.Position + mountOffset_ecef;

        % Sensor orientation = platform orientation * mount orientation offset
        mountAngles = obj.config.mountOrientation;  % [roll; pitch; yaw] in degrees
        R_mount = attitudeToDCM(mountAngles(3), mountAngles(2), mountAngles(1));

        sensorPose.position = sensorPosition;
        sensorPose.orientation = R_body_from_ned * R_mount;
        sensorPose.attitude.heading = platformAttitude.heading + mountAngles(3);
        sensorPose.attitude.pitch = platformAttitude.pitch + mountAngles(2);
        sensorPose.attitude.roll = platformAttitude.roll + mountAngles(1);
    end

    function targetInSensorFrame = transformToSensorFrame(obj, ...
            targetPosECEF, sensorPosECEF, sensorOrientationDCM)
        % TRANSFORMTOSENSORFRAME Transform target position from ECEF to sensor frame
        % Inputs:
        %   targetPosECEF      - Target position in ECEF [x; y; z] meters
        %   sensorPosECEF      - Sensor position in ECEF [x; y; z] meters
        %   sensorOrientationDCM - Sensor orientation DCM (sensor from world)
        % Returns:
        %   targetInSensorFrame - [x; y; z] of target in sensor body frame

        % Relative position in ECEF
        relativeECEF = targetPosECEF - sensorPosECEF;

        % Transform to sensor frame
        targetInSensorFrame = sensorOrientationDCM * relativeECEF;
    end

    function noisyValue = addNoise(obj, value, std)
        % ADDNOISE Add Gaussian noise to a measurement
        % Inputs:
        %   value - True value
        %   std   - Standard deviation of noise
        % Returns:
        %   noisyValue - Value with added noise

        noisyValue = value + std * randn(obj.rng);
    end

    function shouldDetect = shouldUpdate(obj, currentTime)
        % SHOULDUPDATE Check if sensor should generate new measurement
        % Input:
        %   currentTime - Current simulation time in seconds
        % Returns:
        %   shouldDetect - Boolean

        dt = 1.0 / obj.config.updateRate;
        shouldDetect = (currentTime - obj.timeOfLastUpdate) >= dt;
    end

    function detection = detect(obj, targetPose, targetType, sensorPose, sensorAttitude, simTime)
        % DETECT Generate detection if target is in FOV and range
        % This is a template method - subclasses override generateMeasurement
        %
        % Inputs:
        %   targetPose      - Target pose struct with .Position (ECEF)
        %   targetType      - String: 'aircraft', 'missile', etc.
        %   sensorPose      - Sensor pose struct from getSensorPoseInWorld
        %   sensorAttitude  - Sensor attitude struct
        %   simTime         - Current simulation time
        % Returns:
        %   detection - Detection struct (empty if no detection)

        detection = struct();
        detection.valid = false;

        % Check range
        relativePos = targetPose.Position - sensorPose.position;
        range = norm(relativePos);

        if range > obj.config.range
            return;  % Beyond max range
        end

        % Transform to sensor frame for FOV check
        targetInSensorFrame = obj.transformToSensorFrame(...
            targetPose.Position, sensorPose.position, sensorPose.orientation);

        % Check FOV
        if ~obj.checkFOV(targetInSensorFrame, sensorAttitude)
            return;  % Not in FOV
        end

        % Generate measurement (subclass implementation)
        measurement = obj.generateMeasurement(targetInSensorFrame, targetPose, targetType, range);

        % Add metadata
        measurement.sensorId = obj.config.id;
        measurement.sensorName = obj.config.name;
        measurement.sensorType = obj.config.type;
        measurement.time = simTime;
        measurement.targetClass = targetType;
        measurement.valid = true;

        detection = measurement;

        % Store in history
        obj.lastMeasurement = detection;
        obj.measurementHistory{end+1} = detection;
        obj.timeOfLastUpdate = simTime;
    end
end

methods (Abstract)
    % GENERATEMEASUREMENT - Must be implemented by subclasses
    % Returns sensor-specific measurement struct
    measurement = generateMeasurement(obj, targetInSensorFrame, targetPose, targetType, range);
end

methods (Static)
    function R = attitudeToDCM(heading, pitch, roll)
        % ATTITUDEETODCM Convert Euler angles to Direction Cosine Matrix
        % Convention: 3-2-1 (yaw-pitch-roll) rotation sequence
        % Inputs in degrees, returns body-from-nav DCM

        hdg_rad = deg2rad(heading);
        pitch_rad = deg2rad(pitch);
        roll_rad = deg2rad(roll);

        % Individual rotation matrices
        Rz = [cos(hdg_rad), sin(hdg_rad), 0;
              -sin(hdg_rad), cos(hdg_rad), 0;
              0, 0, 1];

        Ry = [cos(pitch_rad), 0, -sin(pitch_rad);
              0, 1, 0;
              sin(pitch_rad), 0, cos(pitch_rad)];

        Rx = [1, 0, 0;
              0, cos(roll_rad), sin(roll_rad);
              0, -sin(roll_rad), cos(roll_rad)];

        % Body from nav: R = Rz' * Ry' * Rx' = (Rx * Ry * Rz)'
        R = (Rx * Ry * Rz)';
    end

    function R = nedRotationMatrix(lat, posECEF)
        % NEDROTATIONMATRIX Compute rotation from ECEF to local NED
        % Input:
        %   lat     - Latitude in degrees
        %   posECEF - Position in ECEF (for longitude)
        % Returns:
        %   R - Rotation matrix (NED from ECEF)

        lat_rad = deg2rad(lat);
        lon_rad = atan2(posECEF(2), posECEF(1));

        % Rotation matrix from ECEF to NED
        R = [-sin(lat_rad)*cos(lon_rad), -sin(lat_rad)*sin(lon_rad), cos(lat_rad);
             -sin(lon_rad),               cos(lon_rad),               0;
             -cos(lat_rad)*cos(lon_rad), -cos(lat_rad)*sin(lon_rad), -sin(lat_rad)];
    end
end

end
