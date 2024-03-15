% 20240216 Calculate vertical tail torque (assuming uniform aerodynamic force distribution)
% Input —— flying_state: Flight state quantities, structure
%         aircraft_para: Basic aircraft parameters, structure
%         design_para: Aircraft design parameters, structure
%         location: Distance from the root of the wing to the position of interest, scalar, in meters
% Output —— torque: Torque, scalar, in Newton-meters
%         bending: Bending moment, scalar, in Newton-meters
%         moment_sum: Sum of external torque from the tip to the position of interest (i.e., integration), scalar, in Newton-meters
% Note: The local external torque at the position of interest cannot be directly given; an approximate value can be provided by the function load_tailV_local_torque

function [torque, moment_sum] = load_tailV_torque(flying_state, aircraft_para, design_para, location)

% Read data
s = aircraft_para.tailscale.height_V; % Wing span (height), in meters
ct = aircraft_para.tailscale.chord_tip_V; % Chord length at the wing tip, in meters
cr = aircraft_para.tailscale.chord_root_V; % Chord length at the wing root, in meters
t = (cr - ct) / s; % Tail taper ratio
x = s - location; % Convert distance from root (fuselage) to distance from tip, in meters
load_case = flying_state.load_case; % Load Case
xhg = design_para.xhg_tailV; % Distance from rudder hinge line to shear center (behind shear center), unit (* chord length)

% Total torque caused by aerodynamic forces (from the tip to this position)
switch load_case
    case 1
        aero_moment_sum = 0;
    case 2
        aero_unit = 34e3 / s; % Unit span aerodynamic force, in N/m
        aero_moment_sum = aero_unit * xhg * (0.5 * t * x^2 + ct * x);
    case 3
        aero_moment_sum = 0;
end

% Total torque (from the tip to this position) and torque itself
moment_sum = aero_moment_sum;
torque = moment_sum;

end
