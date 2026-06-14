function attitude = attitude_utils(velocity, platformType, varargin)
% ATTITUDE_UTILS Calculate aircraft attitude from velocity vector
% Computes heading, pitch, and roll angles from platform velocity.
%
% Inputs:
%   velocity     - 3x1 velocity vector [Vx; Vy; Vz] in m/s (ECEF frame)
%   platformType - String: 'aircraft', 'missile', 'surface', 'subsurface'
%
% Optional Name-Value Pairs:
%   'TurnRate'   - Maximum turn rate in deg/s (default: 3 for aircraft)
%   'BankAngle'  - Bank angle for coordinated turns (default: 30 deg)
%   'PrevAttitude' - Previous attitude for smoothing [hdg; pitch; roll]
%
% Returns:
%   attitude - Struct with fields:
%       .heading - Heading angle in degrees [0, 360)
%       .pitch   - Pitch angle in degrees [-90, 90]
%       .roll    - Roll angle in degrees (coordinated turn estimate)
%
% Notes:
%   - Heading is the direction of horizontal velocity component
%   - Pitch is the climb/descent angle from horizontal
%   - Roll is estimated based on turn rate for coordinated flight

%% Parse optional parameters
p = inputParser;
addRequired(p, 'velocity', @(v) isnumeric(v) && numel(v) >= 3);
addRequired(p, 'platformType', @ischar);
addParameter(p, 'TurnRate', 3, @isnumeric);
addParameter(p, 'BankAngle', 30, @isnumeric);
addParameter(p, 'PrevAttitude', [], @isnumeric);
parse(p, velocity(:), platformType, varargin{:});

turnRateMax = p.Results.TurnRate;
bankAngleDefault = p.Results.BankAngle;
prevAttitude = p.Results.PrevAttitude;

%% Extract velocity components
vx = velocity(1);
vy = velocity(2);
vz = velocity(3);

% Horizontal speed
vHoriz = sqrt(vx^2 + vy^2);
vTotal = sqrt(vx^2 + vy^2 + vz^2);

%% Calculate Heading (azimuth)
% Heading is measured clockwise from North (positive Y in NED)
% For ECEF, we compute the azimuth in the local tangent plane
heading = atan2d(vx, vy);  % atan2(vx, vy) gives heading from North

% Normalize to [0, 360)
if heading < 0
    heading = heading + 360;
end

%% Calculate Pitch (elevation angle)
% Pitch is the angle between velocity vector and horizontal plane
if vTotal > 0.1  % Avoid division by zero
    pitch = asind(vz / vTotal);
else
    pitch = 0;
end

%% Calculate Roll (bank angle)
% For coordinated turns, roll is estimated from heading rate change
% Using simplified model: roll ~ turn rate / g * velocity
roll = 0;  % Default for straight flight

if ~isempty(prevAttitude) && vHoriz > 10  % Velocity threshold
    prevHeading = prevAttitude(1);
    headingChange = heading - prevHeading;

    % Handle wraparound
    if headingChange > 180
        headingChange = headingChange - 360;
    elseif headingChange < -180
        headingChange = headingChange + 360;
    end

    % Estimate roll from heading rate (coordinated turn approximation)
    % tan(roll) ≈ (V * heading_rate) / g
    g = 9.81;  % m/s^2
    dt = 1;    % Assume 1 second update rate (or pass as parameter)

    if abs(headingChange) > 0.1
        % Heading rate in rad/s
        headingRate = deg2rad(headingChange) / dt;

        % Roll angle for coordinated turn
        % Bank angle = atan(V * omega / g)
        rollEstimate = atand(vHoriz * headingRate / g);

        % Limit to maximum bank angle
        roll = min(abs(rollEstimate), bankAngleDefault) * sign(headingChange);
    end
end

%% Platform-specific adjustments
switch lower(platformType)
    case 'missile'
        % Missiles typically have higher pitch authority
        % Roll is often roll-stabilized or different
        roll = roll * 0.5;  % Reduced roll authority

    case 'surface'
        % Surface ships have very limited pitch
        pitch = pitch * 0.1;  % Ships don't pitch much
        roll = 0;  % No intentional banking

    case 'subsurface'
        % Submarines
        pitch = max(-30, min(30, pitch));  % Limit pitch angle
        roll = 0;

    case 'aircraft'
        % Default fixed-wing aircraft model
        % Already calculated above

    otherwise
        % Use default values
end

%% Apply smoothing if previous attitude available
if ~isempty(prevAttitude)
    smoothFactor = 0.7;  % Exponential smoothing

    % Limit sudden changes based on max turn rate
    maxChange = turnRateMax;  % degrees per update

    headingDiff = heading - prevAttitude(1);
    if abs(headingDiff) > 180
        headingDiff = headingDiff - 360 * sign(headingDiff);
    end

    if abs(headingDiff) > maxChange
        heading = prevAttitude(1) + maxChange * sign(headingDiff);
        if heading < 0
            heading = heading + 360;
        elseif heading > 360
            heading = heading - 360;
        end
    end
end

%% Output
attitude = struct();
attitude.heading = heading;
attitude.pitch = pitch;
attitude.roll = roll;
attitude.yaw = heading;  % Alias for consistency

end
