classdef detection_recorder < handle
% DETECTION_RECORDER Handle sensor detection recording to .dat files
% Saves detection data from each sensor to separate files with .dat extension.
%
% File format: CSV with header, one file per sensor
%
% Properties:
%   sensors     - Array of sensor objects
%   outputDir   - Directory for output files
%   fileHandles - Map of sensor IDs to file handles
%   records     - Map of sensor IDs to record counts

properties
    sensors
    outputDir
    fileHandles
    recordCounts
    headers
    enabled
end

methods
    function obj = detection_recorder(sensorConfigs, outputDir, enabled)
        % DETECTIONRECORDER Constructor
        % Inputs:
        %   sensorConfigs - Cell array of sensor configuration structs
        %   outputDir     - Directory for output .dat files
        %   enabled       - Whether file writing is enabled

        if nargin < 3
            enabled = true;
        end

        if nargin < 2 || isempty(outputDir)
            outputDir = 'output';
        end

        obj.outputDir = outputDir;
        obj.enabled = enabled;
        obj.fileHandles = containers.Map('KeyType', 'double', 'ValueType', 'any');
        obj.recordCounts = containers.Map('KeyType', 'double', 'ValueType', 'double');
        obj.headers = containers.Map('KeyType', 'double', 'ValueType', 'char');

        % Create output directory if needed
        if obj.enabled && ~exist(obj.outputDir, 'dir')
            mkdir(obj.outputDir);
        end

        % Setup files for each sensor
        for i = 1:length(sensorConfigs)
            cfg = sensorConfigs(i);
            obj.setupSensorFile(cfg);
        end
    end

    function setupSensorFile(obj, sensorConfig)
        % SETUPSENSORFILE Create output file for a sensor

        sensorId = sensorConfig.id;
        sensorName = sensorConfig.name;
        sensorType = lower(sensorConfig.type);

        % Build filename
        filename = fullfile(obj.outputDir, sprintf('%s_detections.dat', sensorName));

        % Build header based on sensor type
        header = buildHeader(sensorType);

        obj.headers(sensorId) = header;

        if obj.enabled
            fid = fopen(filename, 'w');
            fprintf(fid, '%% Sensor: %s\n', sensorName);
            fprintf(fid, '%% Type: %s\n', sensorType);
            fprintf(fid, '%% Range: %.0f m\n', sensorConfig.range);
            fprintf(fid, '%% Update Rate: %.1f Hz\n', sensorConfig.updateRate);
            fprintf(fid, '%% Generated: %s\n', datestr(now));
            fprintf(fid, '%%\n');
            fprintf(fid, '%s\n', header);

            obj.fileHandles(sensorId) = fid;
        end

        obj.recordCounts(sensorId) = 0;
    end

    function recordDetection(obj, sensorId, detection, targetId, targetName)
        % RECORDDETECTION Record a detection from a sensor
        % Inputs:
        %   sensorId   - Sensor ID
        %   detection  - Detection struct from sensor
        %   targetId   - Target platform ID
        %   targetName - Target platform name

        if ~detection.valid
            return;
        end

        % Increment record count
        obj.recordCounts(sensorId) = obj.recordCounts(sensorId) + 1;

        % Format record based on sensor type
        sensorType = lower(detection.sensorType);
        recordLine = obj.formatRecord(sensorType, detection, targetId, targetName);

        % Write to file
        if obj.enabled && isKey(obj.fileHandles, sensorId)
            fprintf(obj.fileHandles(sensorId), '%s\n', recordLine);
        end
    end

    function line = formatRecord(obj, sensorType, detection, targetId, targetName)
        % FORMATRECORD Format detection as CSV line

        lineParts = {};

        % Common fields
        lineParts{end+1} = sprintf('%.4f', detection.time);
        lineParts{end+1} = sprintf('%d', targetId);
        lineParts{end+1} = targetName;

        % Type-specific fields
        switch sensorType
            case 'radar'
                lineParts{end+1} = sprintf('%.2f', detection.range);
                lineParts{end+1} = sprintf('%.4f', detection.azimuth);
                lineParts{end+1} = sprintf('%.4f', detection.elevation);
                lineParts{end+1} = sprintf('%.2f', detection.rangeRate);
                lineParts{end+1} = sprintf('%.2f', detection.snr);
                lineParts{end+1} = sprintf('%.4f', detection.Pd);
                lineParts{end+1} = sprintf('%.2f', detection.rcsEstimate);

            case 'esm'
                lineParts{end+1} = sprintf('%.4f', detection.azimuth);
                lineParts{end+1} = sprintf('%.4f', detection.elevation);
                lineParts{end+1} = sprintf('%.2f', detection.signalStrength);
                lineParts{end+1} = detection.emitterClass;
                lineParts{end+1} = sprintf('%.4f', detection.pulseInfo.prf);
                lineParts{end+1} = sprintf('%.4f', detection.pulseInfo.pulseWidth);

            case 'ir'
                lineParts{end+1} = sprintf('%.4f', detection.azimuth);
                lineParts{end+1} = sprintf('%.4f', detection.elevation);
                if isfield(detection, 'range') && ~isempty(detection.range)
                    lineParts{end+1} = sprintf('%.2f', detection.range);
                else
                    lineParts{end+1} = 'NaN';
                end
                lineParts{end+1} = sprintf('%.2f', detection.irIntensity);
                lineParts{end+1} = detection.irSignature;

            otherwise
                % Generic: just angles
                if isfield(detection, 'azimuth')
                    lineParts{end+1} = sprintf('%.4f', detection.azimuth);
                end
                if isfield(detection, 'elevation')
                    lineParts{end+1} = sprintf('%.4f', detection.elevation);
                end
        end

        % Ground truth (optional, for analysis)
        if isfield(detection, 'groundTruth')
            gt = detection.groundTruth;
            if isfield(gt, 'range')
                lineParts{end+1} = sprintf('%.2f', gt.range);
            end
            if isfield(gt, 'azimuth')
                lineParts{end+1} = sprintf('%.4f', gt.azimuth);
            end
            if isfield(gt, 'elevation')
                lineParts{end+1} = sprintf('%.4f', gt.elevation);
            end
            if isfield(gt, 'position')
                lineParts{end+1} = sprintf('%.2f', gt.position(1));
                lineParts{end+1} = sprintf('%.2f', gt.position(2));
                lineParts{end+1} = sprintf('%.2f', gt.position(3));
            end
        end

        % Join with commas
        line = strjoin(lineParts, ', ');
    end

    function close(obj)
        % CLOSE Close all file handles

        if obj.enabled
            keys = obj.fileHandles.keys;
            for i = 1:length(keys)
                sensorId = keys{i};
                if isKey(obj.fileHandles, sensorId)
                    fid = obj.fileHandles(sensorId);
                    if fid > 0
                        fclose(fid);
                    end
                end
            end
        end
    end

    function summary = getSummary(obj)
        % GETSUMMARY Get summary of recorded detections
        % Returns map of sensor ID to record count

        summary = obj.recordCounts;
    end
end

methods (Static)
    function header = buildHeader(sensorType)
        % BUILDHEADER Build CSV header for sensor type

        commonFields = 'Time_s, Target_ID, Target_Name';

        switch lower(sensorType)
            case 'radar'
                header = [commonFields, ...
                    ', Range_m, Az_deg, El_deg, RangeRate_mps' ...
                    ', SNR_dB, Pd, RCS_m2' ...
                    ', GT_Range_m, GT_Az_deg, GT_El_deg, GT_PosX_m, GT_PosY_m, GT_PosZ_m'];

            case 'esm'
                header = [commonFields, ...
                    ', Az_deg, El_deg, SignalStrength' ...
                    ', EmitterClass, PRF_Hz, PulseWidth_us' ...
                    ', GT_Az_deg, GT_El_deg, GT_PosX_m, GT_PosY_m, GT_PosZ_m'];

            case 'ir'
                header = [commonFields, ...
                    ', Az_deg, El_deg, RangeEst_m, IRIntensity, Signature' ...
                    ', GT_Range_m, GT_Az_deg, GT_El_deg, GT_PosX_m, GT_PosY_m, GT_PosZ_m'];

            otherwise
                header = [commonFields, ', Az_deg, El_deg'];
        end
    end
end

end
