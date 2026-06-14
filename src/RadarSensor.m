classdef RadarSensor < SensorBase
% RADARSENSOR Active radar sensor model
% Models monostatic radar with range, azimuth, elevation, and range-rate measurements.
%
% Measurement Model:
%   Range: r_measured = r_true + N(0, sigma_r)
%   Azimuth: az_measured = az_true + N(0, sigma_az)
%   Elevation: el_measured = el_true + N(0, sigma_el)
%   Range-rate: rr_measured = rr_true + N(0, sigma_rr)
%
% FOV: Sector or circular scan pattern

methods
    function obj = RadarSensor(config, rngStream)
        % RADARSENSOR Constructor
        obj = obj@SensorBase(config, rngStream);
    end
end

methods
    function measurement = generateMeasurement(obj, targetInSensorFrame, targetPose, targetType, range)
        % GENERATEMEASUREMENT Generate radar measurement
        %
        % Radar measurements:
        %   - Range (meters)
        %   - Azimuth (degrees, from sensor boresight)
        %   - Elevation (degrees, from horizontal)
        %   - Range-rate (m/s, from Doppler)

        % Get true angles to target
        [az_true, el_true, r_true] = obj.cartesianToSpherical(targetInSensorFrame);

        % Add measurement noise
        r_meas = obj.addNoise(r_true, obj.config.noiseParams.rangeStd);
        az_meas = obj.addNoise(az_true, obj.config.noiseParams.azimuthStd);
        el_meas = obj.addNoise(el_true, obj.config.noiseParams.elevationStd);

        % Range-rate measurement (Doppler)
        % Need relative velocity in sensor frame
        % For simplicity, assume target velocity is provided in targetPose
        if isfield(targetPose, 'Velocity') && ~isempty(targetPose.Velocity)
            % Project relative velocity onto line-of-sight
            los = targetInSensorFrame / r_true;  % Unit line-of-sight vector
            rangeRate = dot(targetPose.Velocity, los);  % Assuming sensor is stationary relative
            rangeRate_meas = obj.addNoise(rangeRate, obj.config.noiseParams.rangeRateStd);
        else
            rangeRate_meas = 0;
            rangeRate = 0;
        end

        % Build measurement struct
        measurement = struct();
        measurement.range = r_meas;
        measurement.azimuth = az_meas;
        measurement.elevation = el_meas;
        measurement.rangeRate = rangeRate_meas;

        % Ground truth (for analysis)
        measurement.groundTruth.range = r_true;
        measurement.groundTruth.azimuth = az_true;
        measurement.groundTruth.elevation = el_true;
        measurement.groundTruth.rangeRate = rangeRate;
        measurement.groundTruth.position = targetPose.Position;

        % Target position in sensor frame
        measurement.sensorFramePosition = targetInSensorFrame;

        % Signal-to-Noise Ratio estimation (simplified)
        % SNR = (Pt * G^2 * sigma * lambda^2) / ((4*pi)^3 * R^4 * k * T * B)
        % Simplified model: SNR decreases with R^4
        snrRef = 20;  % Reference SNR at 100km (dB)
        rangeRef = 100000;  % Reference range (m)
        measurement.snr = snrRef - 40 * log10(r_true / rangeRef);

        % Probability of detection (simplified Swerling Model)
        % Pd depends on SNR and false alarm rate
        measurement.Pd = computeProbabilityOfDetection(measurement.snr);

        % Radar cross section estimate (based on target type)
        measurement.rcsEstimate = estimateRCS(targetType);
    end

    function detection = detectRadar(obj, targetPose, targetType, sensorPose, sensorAttitude, simTime)
        % DETECTRADAR Radar-specific detection with range-dependent Pd

        detection = detect@SensorBase(obj, targetPose, targetType, sensorPose, sensorAttitude, simTime);

        if detection.valid
            % Apply detection threshold based on SNR
            if detection.Pd < 0.5
                % Low probability of detection - may miss
                if rand(obj.rng) > detection.Pd
                    detection.valid = false;
                    detection.missReason = 'SNR_below_threshold';
                end
            end
        end
    end
end

methods (Static)
    function Pd = computeProbabilityOfDetection(snr_dB)
        % COMPUTEPROBABILITYOFDETECTION Compute probability of detection
        % Simplified model: Pd = 0.5 + 0.5 * tanh((SNR - SNR_th) / 10)
        % where SNR_th is threshold (typically 13 dB)

        snr_threshold = 13;  % dB

        if snr_dB < 5
            Pd = 0.01;  % Almost certain miss
        elseif snr_dB > 25
            Pd = 0.99;  % Almost certain detection
        else
            % Smooth transition
            Pd = 0.5 + 0.5 * tanh((snr_dB - snr_threshold) / 4);
        end
    end

    function rcs = estimateRCS(targetType)
        % ESTIMATERCS Estimate radar cross section based on target type
        % Returns average RCS in m^2

        switch lower(targetType)
            case 'fighter'
                rcs = 1;     % Small fighter, stealthy
            case 'bomber'
                rcs = 20;    % Large aircraft
            case 'transport'
                rcs = 50;    % Very large
            case 'missile'
                rcs = 0.5;   % Small cross section
            case 'helicopter'
                rcs = 10;    % Rotors add to RCS
            case 'uav'
                rcs = 0.2;    % Small drone
            otherwise
                rcs = 5;     % Default
        end
    end
end

end
