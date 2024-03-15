% 20240215 Calculate horizontal tail inertial loads (gravity)
% Input —— aircraft_para: Basic aircraft parameters, structure
%         location: Distance from the root of the wing to the position of interest, scalar, in meters
% Output —— shear: Shear force, scalar, in Newtons
%         bending: Bending moment, scalar, in Newton-meters
%         load_sum: Sum of tail loads from the wingtip to the position of interest (i.e., integration), scalar, in Newtons
% Note: The local load at the position of interest cannot be directly given, an approximate value can be provided by the function load_tailH_local_inertia

function [shear, bending, load_sum] = load_tailH_inertia(aircraft_para, location)

% Read data
s = (aircraft_para.wingscale.span - aircraft_para.fuslagescale.cabin) / 2; % Half wing span
ct = aircraft_para.wingscale.chord_tip; % Chord length at the wing tip
cr = aircraft_para.wingscale.chord_root; % Chord length at the wing root
t = (cr - ct) / s; % Tail taper ratio
x = aircraft_para.tailscale.span_H / 2 - location; % Convert distance from root to distance from tip, in meters
weight_unit = aircraft_para.weight.empennage_unit; % Unit area tail weight, in kg/m^2

% Total weight (from the tip to this position) (no load factor)
weight_sum = weight_unit * 9.8 * (ct * x + t * x^2 / 2); % Total weight, integral dF = weight_unit * (ct + t * x) * dx;
load_sum = weight_sum; % Total load, no external load, so total load includes only gravity

% Shear & Bending
shear = load_sum; % Shear force
bending_weight = weight_unit * 9.8 * (t * x^3 / 6 + ct * x^2 / 2); % Bending moment caused by gravity, integral of weight_sum
bending = bending_weight; % Bending moment, no external load, so bending is due to gravity only

end
