function plot_trajectories(recorder, config)
% PLOT_TRAJECTORIES Visualize platform trajectories
% Creates a 3D plot showing all platform trajectories in ECEF coordinates.
%
% Inputs:
%   recorder - data_recorder object with recorded trajectory data
%   config   - Scenario configuration struct

figure('Name', 'Platform Trajectories', 'NumberTitle', 'off');
hold on;

colors = struct('ownship', 'b', 'target', 'r');
km = 1000;

legendEntries = {};
legendHandles = [];

for i = 1:length(config.platforms)
    p = config.platforms(i);
    data = recorder.getData(p.id);

    if ~isempty(data)
        % data columns: sample, time, id, lat, lon, alt, vx, vy, vz, h, r, p, y
        [x, y, z] = geodetic2ecef(recorder.wgs, ...
            data(:,4), data(:,5), data(:,6));
        trajData = [x y z];
    else
        % Fallback if no LLA data stored
        trajData = recorder.getData(p.id);
        trajData = trajData(:,4:6); % Extract position columns if available
    end

    if ~isempty(trajData)
        h = plot3(trajData(:,1)/km, trajData(:,2)/km, trajData(:,3)/km, ...
            'color', colors.(p.type), 'LineWidth', 1.5);
        legendEntries{end+1} = [char(p.name) ' (' p.type ')'];
        legendHandles = [legendHandles; h];
    end
end

xlabel('X (km)');
ylabel('Y (km)');
zlabel('Z (km)');
title('Platform Trajectories (ECEF)');
legend(legendHandles, legendEntries, 'Location', 'best');
grid on;
view(3);

end
