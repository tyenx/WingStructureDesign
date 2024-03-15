% 20240215 Calculate local equivalent external torque on the horizontal tail (assuming uniform aerodynamic force distribution)
% Input —— flying_state: Flight state quantities, structure
%         aircraft_para: Basic aircraft parameters, structure
%         design_para: Aircraft design parameters, structure
%         location: Distance from the root of the wing to the position of interest, scalar, in meters
%         LR: Choose left/right side, string
%         mesh_step: The length of the tail wing element divided, scalar, in meters
% Output —— load_local: Local equivalent external torque at the position of interest, scalar, in Newton-meters

function moment_local = load_tailH_local_torque(flying_state, aircraft_para, design_para, location, LR, mesh_step)

% Read data
s = aircraft_para.tailscale.span_H / 2; % Half wing span, in meters
ct = aircraft_para.wingscale.chord_tip; % Chord length at the wing tip
cr = aircraft_para.tailscale.chord_root_H; % Chord length at the wing root, in meters
t = (cr - ct) / s; % Tail taper ratio
x = s - location; % Convert distance from root to distance from tip, in meters
weight_unit = aircraft_para.weight.empennage_unit; % Unit area tail weight, in kg/m^2
load_case = flying_state.load_case; % Load Case
xcg = design_para.xcg_tailH; % Distance between center of gravity and shear center (behind shear center), unit (* chord length)
xhg = design_para.xhg_tailH; % Distance between the elevator hinge line and the shear center (behind shear center), unit (* chord length)

% Torque caused by gravity and aerodynamic forces (both linearly approximated)
weight_moment = (weight_unit * xcg * (ct + t * x)^2 + weight_unit * xcg * (ct + t * (x - mesh_step))^2) / 2 * mesh_step; % Torque caused by gravity
switch load_case
    case 1
        aero_moment = 0;
    case 2
        if strcmp(LR, 'left')
            aero_unit = 52e3 * 0.75 / s; % Unit span aerodynamic force, in N/m
        elseif strcmp(LR, 'right')
            aero_unit = 52e3 * 0.25 / s;
        else
            aero_unit = nan;
        end
        aero_moment = (aero_unit * xhg * (cr - t * location) + aero_unit * xhg * (cr - t * (location + mesh_step))) / 2 * mesh_step;
    case 3
        aero_moment = 0;
end

% Total local moment
moment_local = weight_moment + aero_moment;

end
