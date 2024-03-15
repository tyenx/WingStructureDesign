% 20240207 Calculate local equivalent inertial loads on the wing
% Input —— aircraft_para: Basic aircraft parameters, structure
%         location: Distance from the root of the wing to the position of interest, scalar, in meters
%         mesh_step: The length of the wing element divided, scalar, in meters
% Output —— load_local: Local equivalent inertial load at the position of interest, scalar, in Newtons

function load_local = load_wing_local_inertia(aircraft_para, location, mesh_step)

% Read data
s = (aircraft_para.wingscale.span - aircraft_para.fuslagescale.cabin) / 2; % Half wing span (excluding fuselage width)
ct = aircraft_para.wingscale.chord_tip; % Chord length at the wing tip
cr = aircraft_para.wingscale.chord_root; % Chord length at the wing root
t = (cr - ct) / s; % Wing taper ratio
weight_unit = aircraft_para.weight.wing_unit; % Unit area wing mass, in kg/m^2
engine_location = [aircraft_para.wingscale.distance_engine1;
                   aircraft_para.wingscale.distance_engine1 + aircraft_para.wingscale.distance_engine12;
                   aircraft_para.wingscale.distance_engine1 + aircraft_para.wingscale.distance_engine12 + aircraft_para.wingscale.distance_engine23] - aircraft_para.fuslagescale.width / 2;
                   % Engine installation positions from the wing root, in meters

% Equivalent local load (mass within the element, external load, equivalent to the resultant force at the element near the root)
load_weight = weight_unit * 9.8 * mesh_step * (cr - t * location + cr - t * (location + mesh_step)) / 2; % Mass
if location < engine_location(1) && engine_location(1) - location <= mesh_step
    load_payload = aircraft_para.weight.engine * 9.8; % External load
elseif location < engine_location(2) && engine_location(2) - location <= mesh_step
    load_payload = aircraft_para.weight.engine * 9.8;
elseif location < engine_location(3) && engine_location(3) - location <= mesh_step
    load_payload = aircraft_para.weight.engine * 9.8;
else
    load_payload = 0;
end
load_local = -(load_weight + load_payload);

end
