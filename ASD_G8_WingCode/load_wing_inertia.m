% 20240207 Calculate wing inertial loads (including gravity and external loads)
% Input —— aircraft_para: Basic aircraft parameters, structure
%         location: Distance from the root of the wing to the position of interest, scalar, in meters
% Output —— shear: Shear force, scalar, in Newtons
%         bending: Bending moment, scalar, in Newton-meters
%         load_sum: Sum of wing loads from the tip to the position of interest (i.e., integration), scalar, in Newtons

function [shear, bending, load_sum] = load_wing_inertia(aircraft_para, location)

% Read data
s = (aircraft_para.wingscale.span - aircraft_para.fuslagescale.cabin) / 2; % Half wing span (excluding fuselage width)
ct = aircraft_para.wingscale.chord_tip; % Chord length at the wing tip
cr = aircraft_para.wingscale.chord_root; % Chord length at the wing root
t = (cr - ct) / s; % Wing taper ratio
x = s - location; % Convert distance from root to distance from tip, in meters
weight_unit = aircraft_para.weight.wing_unit; % Unit area wing mass, in kg/m^2
engine_location = [aircraft_para.wingscale.distance_engine1;
                   aircraft_para.wingscale.distance_engine1 + aircraft_para.wingscale.distance_engine12;
                   aircraft_para.wingscale.distance_engine1 + aircraft_para.wingscale.distance_engine12 + aircraft_para.wingscale.distance_engine23] - aircraft_para.fuslagescale.width / 2;
                   % Engine installation positions from the wing root, in meters

% Total load (from the tip to this position) (no load factor)
if location < engine_location(1)
    payload_sum = 3 * aircraft_para.weight.engine * 9.8; % Total external load
elseif location < engine_location(2)
    payload_sum = 2 * aircraft_para.weight.engine * 9.8;
elseif location < engine_location(3)
    payload_sum = 1 * aircraft_para.weight.engine * 9.8;
else
    payload_sum = 0;
end
weight_sum = weight_unit * 9.8 * (ct * x + t * x^2 / 2); % Total mass, integral dF = weight_unit * (ct + t * x) * dx;
load_sum = -(payload_sum + weight_sum); % Total load

% Shear & Bending
shear = load_sum; % Shear force
bending_weight = weight_unit * 9.8 * (t * x^3 / 6 + ct * x^2 / 2); % Bending moment caused by gravity, integral of weight_sum
if location < engine_location(1)
    bending_payload = aircraft_para.weight.engine * 9.8 * (engine_location(1) + engine_location(2) + engine_location(3) - 3 * location); % Bending moment caused by external load
elseif location < engine_location(2)
    bending_payload = aircraft_para.weight.engine * 9.8 * (engine_location(2) + engine_location(3) - 2 * location);
elseif location < engine_location(3)
    bending_payload = aircraft_para.weight.engine * 9.8 * (engine_location(3) - location);
else
    bending_payload = 0;
end
bending = -(bending_weight + bending_payload); % Bending moment

end
