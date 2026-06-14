function waypoints_ecef = waypoint_utils(waypoints, coordSystem, wgs)
% WAYPOINT_UTILS Convert waypoints to ECEF coordinates
% Converts waypoint coordinates from the specified coordinate system
% to ECEF (Earth-Centered, Earth-Fixed) cartesian coordinates.
%
% Inputs:
%   waypoints   - Nx3 matrix of waypoints
%   coordSystem - 'lla' for geodetic (lat, lon, alt) or 'ecef' for cartesian
%   wgs         - WGS84 ellipsoid reference (from wgs84Ellipsoid)
%
% Returns:
%   waypoints_ecef - Nx3 matrix of ECEF coordinates [x, y, z] in meters

if nargin < 3
    wgs = wgs84Ellipsoid('kilometer');
end

switch lower(coordSystem)
    case 'lla'
        % Geodetic coordinates: lat (deg), lon (deg), alt (m)
        n = size(waypoints, 1);
        waypoints_ecef = zeros(n, 3);
        for i = 1:n
            [x, y, z] = geodetic2ecef(wgs, ...
                waypoints(i,1), waypoints(i,2), waypoints(i,3));
            waypoints_ecef(i, :) = [x y z];
        end

    case 'ecef'
        % Already in ECEF cartesian coordinates
        waypoints_ecef = waypoints;

    otherwise
        error('Coordinate system must be ''lla'' or ''ecef''');
end

end
