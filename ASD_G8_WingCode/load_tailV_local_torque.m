% 20240216 Calculate local equivalent external torque on the vertical tail (assuming uniform aerodynamic force distribution)
% Input —— flying_state: Flight state quantities, structure
%         aircraft_para: Basic aircraft parameters, structure
%         design_para: Aircraft design parameters, structure
%         location: Distance from the root of the wing to the position of interest, scalar, in meters
%         mesh_step: The length of the tail wing element divided, scalar, in meters
% Output —— load_local: Local equivalent external torque at the position of interest, scalar, in Newton-meters

function moment_local = load_tailV_local_torque(flying_state, aircraft_para, design_para, location, mesh_step)

% Read data
s = aircraft_para.tailscale.height_V; % Span (height), in meters
cr = aircraft_para.tailscale.chord_root_V; % Chord length at the root, in meters
t = (aircraft_para.tailscale.chord_root_V - aircraft_para.tailscale.chord_tip_V) / s; % Tail taper ratio
load_case = flying_state.load_case; % Load Case
xhg = design_para.xhg_tailV; % Distance between the rudder hinge line and the shear center (behind shear center), in chord length units

% Torque caused by aerodynamic forces (linear approximation)
switch load_case
    case 1
        aero_moment = 0;
    case 2
        aero_unit = 34e3 / s; % Unit span aerodynamic force, in N/m
        aero_moment = (aero_unit * xhg * (cr - t * location) + aero_unit * xhg * (cr - t * (location + mesh_step))) / 2 * mesh_step;
    case 3
        aero_moment = 0;
end

% Total local moment
moment_local = aero_moment;

end
