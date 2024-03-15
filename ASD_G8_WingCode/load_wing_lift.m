% 20240207 Calculate wing lift loads (assuming elliptical distribution)
% Input —— flying_state: Flight state quantities, structure
%         aircraft_para: Basic aircraft parameters, structure
%         location: Distance from the root of the wing to the position of interest, scalar, in meters
% Output —— shear: Shear force, scalar, in Newtons
%         bending: Bending moment, scalar, in Newton-meters
%         lift_local: Lift at the position of interest, scalar, in N/m
%         lift_sum: Sum of lift from the tip to the position of interest (i.e., integration), scalar, in Newtons

function [shear, bending, lift_local, lift_sum] = load_wing_lift(flying_state, aircraft_para, design_para, location)

% Read data
s = (aircraft_para.wingscale.span - aircraft_para.fuslagescale.cabin) / 2; % Half wing span (excluding fuselage width)
AOA = flying_state.AoA + design_para.wing_install_AOA; % Angle of attack, in degrees
L0 = 0.5 * flying_state.air_density * flying_state.speed^2 * aircraft_para.wingscale.chord_root * (aircraft_para.aerofoil.cl0 + AOA * aircraft_para.aerofoil.dcl_dalfa); % Lift at the wing root, in N/m

% Lift function and moment function
dlift_fun = @(y) L0 * sqrt(1 - y.^2 / s^2); % Lift function, y is the distance from the root
dmoment_fun = @(y, x) L0 * sqrt(1 - y.^2 / s^2) .* (y - x); % Moment function due to lift, x is the distance from the reference point to the root

% Lift at this position and total lift (from the tip to this position)
lift_local = L0 * sqrt(1 - location^2 / s^2);
lift_sum = integral(dlift_fun, location, s);

% Shear & Bending
shear = lift_sum;
bending = integral(@(y) dmoment_fun(y, location), location, s);

end
