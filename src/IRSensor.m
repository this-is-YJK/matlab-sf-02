classdef IRSensor < SensorBase
% IRSENSOR Infrared Search and Track (IRST) sensor model
% Passive sensor that detects thermal signatures from platforms.
% Provides angle measurements with optional range estimation.
%
% Measurement Model:
%   Azimuth: az_measured = az_true + N(0, sigma_az)
%   Elevation: el_measured = el_true + N(0, sigma_el)
%
% Detection depends on:
%   - IR signature (engine heat, skin friction)
%   - Atmospheric conditions
%   - Aspect angle (head-on vs tail-on view)

methods
    function obj = IRSensor(config, rngStream)
        % IRSENSOR Constructor
        obj = obj@SensorBase(config, rngStream);
    end
end

methods
    function measurement = generateMeasurement(obj, targetInSensorFrame, targetPose, targetType, range)
        % GENERATEMEASUREMENT Generate IR measurement
        %
        % IR provides:
        %   - Azimuth angle (degrees)
        %   - Elevation angle (degrees)
        %   - IR intensity (relative units)
        %   - Optional range estimate (low accuracy)

        % Get true angles to target
        [az_true, el_true, r_true] = obj.cartesianToSpherical(targetInSensorFrame);

        % Add measurement noise
        az_meas = obj.addNoise(az_true, obj.config.noiseParams.azimuthStd);
        el_meas = obj.addNoise(el_true, obj.config.noiseParams.elevationStd);

        % Compute IR signature based on target type and aspect
        irSignature = computeIRSignature(targetType, targetInSensorFrame, r_true);

        % Build measurement struct
        measurement = struct();
        measurement.azimuth = az_meas;
        measurement.elevation = el_meas;

        % Range estimation (if supported)
        if isfield(obj.config, 'noiseParams') && ...
           isfield(obj.config.noiseParams, 'rangeEstimate') && ...
           obj.config.noiseParams.rangeEstimate
            % Rough range estimate based on intensity
            rangeSig = irSignature.intensity;
            if rangeSig > 0
                r_est = obj.config.range * sqrt(irSignature.referenceIntensity / rangeSig);
                r_est = min(r_est, obj.config.range);
                r_est = max(r_est, 1000);
            else
                r_est = obj.config.range;
            end
            % Range estimate noise is large (10-20% error)
            measurement.range = obj.addNoise(r_est, 0.15 * r_est);
        else
            measurement.range = [];
        end

        % IR detection quality
        measurement.irIntensity = irSignature.intensity;
        measurement.irSignature = irSignature.type;

        % Ground truth
        measurement.groundTruth.azimuth = az_true;
        measurement.groundTruth.elevation = el_true;
        measurement.groundTruth.position = targetPose.Position;
        measurement.sensorFramePosition = targetInSensorFrame;
        measurement.irRange = r_true;
    end

    function detection = detectIR(obj, targetPose, targetType, sensorPose, sensorAttitude, simTime)
        % DETECTIR IR-specific detection with signature-based Pd

        detection = detect@SensorBase(obj, targetPose, targetType, sensorPose, sensorAttitude, simTime);

        if detection.valid
            % Check IR signature threshold
            irThreshold = 0.1;  % Minimum signature for detection
            if detection.irIntensity < irThreshold
                detection.valid = false;
                detection.missReason = 'IR_signature_below_threshold';
            end
        end
    end
end

methods (Static)
    function sig = computeIRSignature(targetType, targetPos, range)
        % COMPUTEIRSIGNAL Compute IR signature intensity
        %
        % Returns struct with:
        %   intensity - Relative intensity (decreases with range^2)
        %   type       - Signature type classification
        %   referenceIntensity - Standard intensity for range calculations

        % Base IR signature (arbitrary units at 10km range)
        baseSig = getBaseIRSignature(targetType);

        % Atmospheric attenuation (simplified)
        % IR attenuates with water vapor, CO2 absorption
        % Typical: ~0.5 dB/km at low altitude
        attenuation = 0.5;  % dB/km
        range_km = range / 1000;
        atmosphericLoss = attenuation * range_km;

        % Range-dependent intensity (inverse square + attenuation)
        refRange = 10000;  % Reference range for base signature
        intensity = baseSig.intensity * (refRange / range)^2 * 10^(-atmosphericLoss / 10);

        sig.intensity = intensity;
        sig.type = baseSig.type;
        sig.referenceIntensity = baseSig.intensity;
        sig.aspect = 'unknown';  % Could be computed from aspect angle
    end

    function baseSig = getBaseIRSignature(targetType)
        % GETBASEIRSIGNAL Get base IR signature for target type

        switch lower(targetType)
            case 'fighter'
                % Afterburning fighter: very hot
                baseSig.intensity = 100;
                baseSig.type = 'jet_engine';
            case 'bomber'
                % Multiple engines
                baseSig.intensity = 80;
                baseSig.type = 'jet_engine';
            case 'transport'
                % Large but lower temperature
                baseSig.intensity = 40;
                baseSig.type = 'turbofan';
            case 'helicopter'
                % Hot exhaust
                baseSig.intensity = 30;
                baseSig.type = 'turboshaft';
            case 'missile'
                % Very hot rocket plume
                baseSig.intensity = 200;
                baseSig.type = 'rocket';
            case 'uav'
                % Small engine
                baseSig.intensity = 10;
                baseSig.type = 'small_engine';
            case 'surface_combatant'
                % Ship exhaust
                baseSig.intensity = 20;
                baseSig.type = 'stack';
            otherwise
                baseSig.intensity = 50;
                baseSig.type = 'unknown';
        end
    end
end

end
