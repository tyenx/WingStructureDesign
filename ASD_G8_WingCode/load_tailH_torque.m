% 20240215 Calculate horizontal tail torque (assuming uniform aerodynamic force distribution)
% Input —— flying_state: Flight state quantities, structure
%         aircraft_para: Basic aircraft parameters, structure
%         design_para: Aircraft design parameters, structure
%         location: Distance from the root of the wing to the position of interest, scalar, in meters
%         LR: Choose left/right side, string
% Output —— torque: Torque, scalar, in Newton-meters
%         bending: Bending moment, scalar, in Newton-meters
%         moment_sum: Sum of external moments on the tail from the tip to the position of interest (i.e., integration), scalar, in Newton-meters
% Note: The local external moment at the position of interest cannot be directly given, an approximate value can be provided by the function load_tailH_local_torque

function [torque, moment_sum] = load_tailH_torque(flying_state, aircraft_para, design_para, location, LR)

% Read data
s = aircraft_para.tailscale.span_H / 2; % Half wing span, in meters
ct = aircraft_para.tailscale.chord_tip_H; % Chord length at the wing tip, in meters
cr = aircraft_para.tailscale.chord_root_H; % Chord length at the wing root, in meters
t = (cr - ct) / s; % Tail taper ratio
x = s - location; % Convert distance from root to distance from tip, in meters
weight_unit = aircraft_para.weight.empennage_unit; % Unit area tail weight, in kg/m^2
load_case = flying_state.load_case; % Load Case
xcg = design_para.xcg_tailH; % Distance between the center of gravity and the shear center (positive in front of the shear center), unit (* chord length)
xhg = design_para.xhg_tailH; % Distance between the elevator hinge line and the shear center (positive in front of the shear center), unit (* chord length)

% Total moment caused by gravity (from the tip to this position)
weight_moment_sum = weight_unit * xcg * (ct^2 * x + ct * t * x^2 + t^2 * x^3 / 3); % Integral dM = dF * xcg * (ct + t * x) = weight_unit * xcg * (ct + t * x)^2 * dx

% Total moment caused by aerodynamic forces (from the tip to this position)
switch load_case
    case 1
        aero_moment_sum = 0;
    case 2
        if strcmp(LR, 'left')
            aero_unit = 52e3 * 0.75 / s; % Unit span aerodynamic force, in N/m
        elseif strcmp(LR, 'right')
            aero_unit = 52e3 * 0.25 / s;
        else
            aero_unit = nan;
        end
        aero_moment_sum = aero_unit * xhg * (0.5 * t * x^2 + ct * x);
    case 3
        aero_moment_sum = 0;
end

% Total moment (from the tip to this position) and torque
moment_sum = weight_moment_sum + aero_moment_sum;
torque = moment_sum;

end
