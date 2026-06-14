classdef ESMSensor < SensorBase
% ESMSENSOR Electronic Support Measures (Radar Warning Receiver) sensor
% Passive sensor that detects radar emissions from other platforms.
% Provides angle-of-arrival (AOA) measurements only.
%
% Measurement Model:
%   Azimuth: az_measured = az_true + N(0, sigma_az)
%   Elevation: el_measured = el_true + N(0, sigma_el)
%
% ESM detects platforms that have ACTIVE radar (emitters).
% Targets without active emitters are invisible to ESM.

methods
    function obj = ESMSensor(config, rngStream)
        % ESMSENSOR Constructor
        obj = obj@SensorBase(config, rngStream);
    end
end

methods
    function measurement = generateMeasurement(obj, targetInSensorFrame, targetPose, targetType, range)
        % GENERATEMEASUREMENT Generate ESM measurement (angle-only)
        %
        % ESM provides:
        %   - Azimuth angle (degrees)
        %   - Elevation angle (degrees)
        %   - Signal strength indicator (relative)
        %   - Emitter classification (if possible)

        % Get true angles to target
        [az_true, el_true, r_true] = obj.cartesianToSpherical(targetInSensorFrame);

        % Add measurement noise
        az_meas = obj.addNoise(az_true, obj.config.noiseParams.azimuthStd);
        el_meas = obj.addNoise(el_true, obj.config.noiseParams.elevationStd);

        % Build measurement struct
        measurement = struct();
        measurement.azimuth = az_meas;
        measurement.elevation = el_meas;
        measurement.range = [];  % No range measurement

        % Signal strength (decreases with range^2 for RF)
        refStrength = 100;  % Reference at 100km
        strength = refStrength * (100000 / r_true)^2;
        measurement.signalStrength = min(strength, 1000);  % Cap at max

        % Ground truth
        measurement.groundTruth.azimuth = az_true;
        measurement.groundTruth.elevation = el_true;
        measurement.groundTruth.position = targetPose.Position;
        measurement.sensorFramePosition = targetInSensorFrame;

        % Emitter classification (based on assumed target radar)
        measurement.emitterClass = classifyEmitter(targetType);

        % Pulse characteristics (placeholder for detailed analysis)
        measurement.pulseInfo.pulseWidth = 0.1 + 0.9 * rand(obj.rng);  % microseconds
        measurement.pulseInfo.prf = 1000 + 9000 * rand(obj.rng);  % Hz
        measurement.pulseInfo.frequency = 9e9;  % X-band default

        % Emitter bearing line (unit vector in sensor frame)
        bearingLine = targetInSensorFrame / r_true;
        measurement.bearingLine = bearingLine;
    end

    function detection = detectESM(obj, targetPose, targetType, sensorPose, sensorAttitude, simTime, targetHasEmitter)
        % DETECTESM ESM-specific detection requiring active emitter
        %
        % Additional Input:
        %   targetHasEmitter - Boolean indicating if target has active radar

        if ~targetHasEmitter
            detection = struct();
            detection.valid = false;
            detection.missReason = 'No_active_emitter';
            return;
        end

        detection = detect@SensorBase(obj, targetPose, targetType, sensorPose, sensorAttitude, simTime);

        if detection.valid
            % Check signal strength threshold
            if detection.signalStrength < 10  % Minimum detectable signal
                detection.valid = false;
                detection.missReason = 'Signal_below_threshold';
            end
        end
    end
end

methods (Static)
    function emitterClass = classifyEmitter(targetType)
        % CLASSIFYEMITTER Classify emitter type from target type

        switch lower(targetType)
            case 'fighter'
                emitterClass = 'fire_control_radar';
            case 'bomber'
                emitterClass = 'search_radar';
            case 'awacs'
                emitterClass = 'surveillance';
            case 'surface_combatant'
                emitterClass = 'surface_search';
            case 'sam_site'
                emitterClass = 'acquisition_radar';
            otherwise
                emitterClass = 'unknown';
        end
    end
end

end
