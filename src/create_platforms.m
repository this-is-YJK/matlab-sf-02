function platforms = create_platforms(scenario, config)
% CREATE_PLATFORMS Create platform objects in tracking scenario
% Creates all platforms defined in the configuration and attaches
% waypoint trajectories.
%
% Inputs:
%   scenario - trackingScenario object
%   config   - Configuration struct from scenario_config()
%
% Returns:
%   platforms - Cell array of platform objects

wgs = wgs84Ellipsoid('kilometer');
nPlatforms = length(config.platforms);
platforms = cell(1, nPlatforms);

for i = 1:nPlatforms
    p = config.platforms(i);

    % Create platform
    platforms{i} = platform(scenario);

    % Convert waypoints to ECEF if needed
    wp_ecef = waypoint_utils(p.waypoints, p.coordSystem, wgs);

    % Create and attach trajectory
    platforms{i}.Trajectory = waypointTrajectory(wp_ecef, p.times);
end

end
