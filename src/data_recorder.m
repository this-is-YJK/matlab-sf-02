classdef data_recorder < handle
% DATA_RECORDER Handle data recording during scenario simulation
% Manages file output and in-memory data storage for trajectory data.
% Calculates and records platform attitude (heading, pitch, roll).
%
% Properties:
%   fileID      - File identifier for output file
%   filename    - Output filename
%   header      - CSV header string
%   enabled     - Whether recording is enabled
%   data        - Cell array storing recorded data per platform
%   prevAttitude - Previous attitude values for smoothing
%
% Methods:
%   DataRecorder - Constructor
%   record       - Record data for a platform at current time step
%   close        - Close the output file
%   getData      - Get recorded data for a specific platform

properties
    fileID
    filename
    header
    enabled
    data
    wgs
    prevAttitude  % Store previous attitude per platform for smoothing
end

methods
    function obj = data_recorder(config)
        % DATARECORDER Constructor
        % Input:
        %   config - Scenario configuration struct
        obj.enabled = config.output.enabled;
        obj.filename = config.output.filename;
        obj.header = config.output.header;
        obj.wgs = wgs84Ellipsoid('kilometer');
        obj.data = {};
        obj.prevAttitude = containers.Map('KeyType', 'double', 'ValueType', 'any');

        if obj.enabled
            obj.fileID = fopen(obj.filename, 'w');
            fprintf(obj.fileID, '%s\n', obj.header);
        end
    end

    function record(obj, platform, platformConfig, sampleNo, simTime)
        % RECORD Record platform state at current time step
        % Inputs:
        %   platform       - Platform object
        %   platformConfig - Platform configuration struct
        %   sampleNo       - Current sample number
        %   simTime        - Current simulation time

        % Get platform pose (ECEF coordinates)
        p = pose(platform);

        % Convert ECEF to geodetic
        [lat, lon, alt] = ecef2geodetic(obj.wgs, ...
            p.Position(1), p.Position(2), p.Position(3));

        % Calculate attitude from velocity
        platformId = platformConfig.id;

        if isKey(obj.prevAttitude, platformId)
            prevAtt = obj.prevAttitude(platformId);
            att = attitude_utils(p.Velocity, 'aircraft', 'PrevAttitude', prevAtt);
        else
            att = attitude_utils(p.Velocity, 'aircraft');
        end

        % Store current attitude for next iteration
        obj.prevAttitude(platformId) = [att.heading; att.pitch; att.roll];

        % Build data record
        % Columns: sample, time, id, lat, lon, alt, vx, vy, vz, heading, roll, pitch, yaw
        record = [sampleNo simTime platformConfig.id lat lon alt ...
                  p.Velocity(1) p.Velocity(2) p.Velocity(3) ...
                  att.heading att.roll att.pitch att.heading];

        % Store in memory
        idx = platformConfig.id + 1;
        if idx > length(obj.data)
            obj.data{idx} = [];
        end
        obj.data{idx} = [obj.data{idx}; record];

        % Write to file
        if obj.enabled
            fprintf(obj.fileID, '%d, %.4f, %d, %s, %.6f, %.6f, %.6f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n', ...
                sampleNo, simTime, platformConfig.id, platformConfig.name, ...
                lat, lon, alt, p.Velocity(1), p.Velocity(2), p.Velocity(3), ...
                att.heading, att.roll, att.pitch, att.heading);
        end
    end

    function close(obj)
        % CLOSE Close the output file
        if obj.enabled && obj.fileID > 0
            fclose(obj.fileID);
        end
    end

    function d = getData(obj, platformId)
        % GETDATA Retrieve recorded data for a platform
        % Input:
        %   platformId - Platform ID
        % Returns:
        %   d - Matrix of recorded data for the platform
        idx = platformId + 1;
        if idx <= length(obj.data)
            d = obj.data{idx};
        else
            d = [];
        end
    end
end

end
