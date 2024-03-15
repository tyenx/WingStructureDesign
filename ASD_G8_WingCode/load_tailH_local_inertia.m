% 20240215 Calculate local equivalent inertial loads on the horizontal tail
% Input —— aircraft_para: Basic aircraft parameters, structure
%         location: Distance from the root of the wing to the position of interest, scalar, in meters
%         mesh_step: Length of the tail segment for load calculation, scalar, in meters
% Output —— load_local: Local equivalent inertial load at the position of interest, scalar, in Newtons

function load_local = load_tailH_local_inertia(aircraft_para, location, mesh_step)

% Read data
s = (aircraft_para.wingscale.span - aircraft_para.fuslagescale.cabin) / 2; % Half wing span (excluding fuselage width)
ct = aircraft_para.wingscale.chord_tip; % Chord length at the wing tip
cr = aircraft_para.wingscale.chord_root; % Chord length at the wing root
t = (cr - ct) / s; % Wing taper ratio
weight_unit = aircraft_para.weight.empennage_unit; % Unit area tail weight, in kg/m^2

% Equivalent local load (mass within the element, external load, equivalent to the force at the element closer to the root)
load_weight = weight_unit * 9.8 * mesh_step * (cr - t * location + cr - t * (location + mesh_step)) / 2; % Mass
load_local = load_weight; % Local load, no external load, so the load includes only gravity

end
