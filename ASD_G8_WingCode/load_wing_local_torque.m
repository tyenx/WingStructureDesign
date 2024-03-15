% 20240207 Calculate Wing Local Equivalent Torque
% Input —— flying_state: Flight state variables, structure
%         aircraft_para: Basic aircraft parameters, structure
%         design_para: Aircraft design parameters, structure
%         location: The position from the wing root, scalar, unit m
%         mesh_step: Discretization length for wing elements, scalar, unit m
% Output —— load_local: The local equivalent external torque at the specified position, scalar, unit Nm

function moment_local = load_wing_local_torque(flying_state, aircraft_para, design_para, location, mesh_step)

% Read data
s = (aircraft_para.wingscale.span - aircraft_para.fuslagescale.cabin) / 2; % Semi-span (excluding fuselage width)
ct = aircraft_para.wingscale.chord_tip; % Tip chord length
cr = aircraft_para.wingscale.chord_root; % Root chord length
t = (cr - ct) / s; % Wing taper ratio
x = s - location; % Conversion from distance from root to distance from tip, unit m
weight_unit = aircraft_para.weight.wing_unit; % Wing mass per unit area, unit kg/m^2
engine_location = [aircraft_para.wingscale.distance_engine1;
                   aircraft_para.wingscale.distance_engine1 + aircraft_para.wingscale.distance_engine12;
                   aircraft_para.wingscale.distance_engine1 + aircraft_para.wingscale.distance_engine12 + aircraft_para.wingscale.distance_engine23] - aircraft_para.fuslagescale.width / 2;
                   % Engine installation positions relative to the wing root, unit m
xcg = design_para.xcg; % Distance between the center of gravity and the shear center (positive if behind), unit (* chord length)
xac = design_para.xac; % Distance between the aerodynamic center and the shear center (positive if ahead), unit (* chord length)
xeg = design_para.xeg; % Distance between the engine installation position and the shear center (positive if ahead), unit (* chord length)
heg = design_para.heg; % Height difference between the engine installation position and the shear center (positive if below), unit (* chord length)
thrust = aircraft_para.other.power / flying_state.speed; % Engine thrust, questionable if directly power/speed?
AOA = flying_state.AoA + design_para.wing_install_AOA; % Angle of attack, unit °
L0 = 0.5 * flying_state.air_density * flying_state.speed^2 * aircraft_para.wingscale.chord_root * (aircraft_para.aerofoil.cl0 + AOA * aircraft_para.aerofoil.dcl_dalfa); % Lift at root chord, unit N/m
M0 = 0.5 * flying_state.air_density * flying_state.speed^2 * aircraft_para.wingscale.chord_root^2 * (aircraft_para.aerofoil.cm0 + AOA * aircraft_para.aerofoil.dcm_dalfa); % Lift moment at root chord, unit Nm/m

% Gravity and lift induced torques (linear approximation)
weight_moment = (weight_unit * xcg * (ct + t * x)^2 + weight_unit * xcg * (ct + t * (x - mesh_step))^2) / 2 * mesh_step; % Gravity-induced torque, average over the mesh step
lift_moment1 = (L0 * sqrt(1 - location^2 / s^2) * xac * (cr - t * location) + L0 * sqrt(1 - (location + mesh_step)^2 / s^2) * xac * (cr - t * (location + mesh_step))) / 2 * mesh_step;
lift_moment2 = (M0 * sqrt(1 - location^2 / s^2) + M0 * sqrt(1 - (location + mesh_step)^2 / s^2)) / 2 * mesh_step;

% External load and engine thrust induced torques
if location < engine_location(1) && engine_location(1) - location <= mesh_step
    payload_moment = 3 * aircraft_para.weight.engine * 9.8 * xeg; % External load-induced torque
    thrust_moment = 3 * thrust * heg; % Engine thrust-induced torque
elseif location < engine_location(2) && engine_location(2) - location <= mesh_step
    payload_moment = 2 * aircraft_para.weight.engine * 9.8 * xeg;
    thrust_moment = 2 * thrust * heg;
elseif location < engine_location(3) && engine_location(3) - location <= mesh_step
    payload_moment = 1 * aircraft_para.weight.engine * 9.8 * xeg;
    thrust_moment = 1 * thrust * heg;
else
    payload_moment = 0;
    thrust_moment = 0;
end

moment_local = weight_moment - thrust_moment + payload_moment - lift_moment1 + lift_moment2;

end
