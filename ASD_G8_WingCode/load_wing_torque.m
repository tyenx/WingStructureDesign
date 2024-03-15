% 20240207 Calculate Wing Torque
% Input —— flying_state: Flight state variables, structure
%         aircraft_para: Basic aircraft parameters, structure
%         design_para: Aircraft design parameters, structure
%         location: The position from the wing root, scalar, unit m
% Output —— torque: Torque, scalar, unit Nm
%          bending: Bending moment, scalar, unit Nm
%          moment_sum: Sum of external moments from the wingtip to the specified position, scalar, unit Nm

function [torque, moment_sum] = load_wing_torque(flying_state, aircraft_para, design_para, location)

% Read data
s = (aircraft_para.wingscale.span - aircraft_para.fuslagescale.cabin) / 2; % Semi-span (excluding fuselage width)
ct = aircraft_para.wingscale.chord_tip; % Tip chord length
cr = aircraft_para.wingscale.chord_root; % Root chord length
t = (aircraft_para.wingscale.chord_root - aircraft_para.wingscale.chord_tip) / s; % Wing taper ratio
x = s - location; % Conversion from distance from root to distance from tip, unit m
weight_unit = aircraft_para.weight.wing_unit; % Wing mass per unit area, unit kg/m^2
engine_location = [aircraft_para.wingscale.distance_engine1;
                   aircraft_para.wingscale.distance_engine1 + aircraft_para.wingscale.distance_engine12;
                   aircraft_para.wingscale.distance_engine1 + aircraft_para.wingscale.distance_engine12 + aircraft_para.wingscale.distance_engine23] - aircraft_para.fuslagescale.width / 2;
                   % Engine installation positions relative to the wing root, unit m
xcg = design_para.xcg; % Distance between the center of gravity and the shear center (positive if ahead), unit (* chord length)
xac = design_para.xac; % Distance between the aerodynamic center and the shear center (positive if ahead), unit (* chord length)
xeg = design_para.xeg; % Distance between the engine installation position and the shear center (positive if ahead), unit (* chord length)
heg = design_para.heg; % Height difference between the engine installation position and the shear center (positive if below), unit (* chord length)
thrust = aircraft_para.other.power / flying_state.speed; % Engine thrust, questioning if directly power/speed?
AOA = flying_state.AoA + design_para.wing_install_AOA; % Angle of attack, unit °
L0 = 0.5 * flying_state.air_density * flying_state.speed^2 * aircraft_para.wingscale.chord_root * (aircraft_para.aerofoil.cl0 + AOA * aircraft_para.aerofoil.dcl_dalfa); % Lift at root chord, unit N/m
M0 = 0.5 * flying_state.air_density * flying_state.speed^2 * aircraft_para.wingscale.chord_root^2 * (aircraft_para.aerofoil.cm0 + AOA * aircraft_para.aerofoil.dcm_dalfa); % Lift moment at root chord, unit Nm/m

% Total moment caused by gravity from the tip to the specified position
weight_moment_sum = weight_unit * xcg * (ct^2 * x + ct * t * x^2 + t^2 * x^3 / 3); % Integral dM = dF * xcg * (ct + t * x) = weight_unit * xcg * (ct + t * x)^2 * dx

% Total moment caused by lift from the tip to the specified position
dlift_moment1 = @(y) L0 * sqrt(1 - y.^2 / s^2) .* xac .* (cr - t * y); % Moment due to lift offset from the shear center
dlift_moment2 = @(y) M0 * sqrt(1 - y.^2 / s^2); % Lift moment, questioning if elliptical distribution?
lift_moment_sum = integral(dlift_moment1, location, s) + integral(dlift_moment2, location, s);

% Total moment caused by external loads and engine thrust from the tip to the specified position
if location < engine_location(1)
    payload_moment_sum = 3 * aircraft_para.weight.engine * 9.8 * xeg; % Moment due to external loads
    thrust_moment_sum = 3 * thrust * heg; % Moment due to engine thrust
elseif location < engine_location(2)
    payload_moment_sum = 2 * aircraft_para.weight.engine * 9.8 * xeg;
    thrust_moment_sum = 2 * thrust * heg;
elseif location < engine_location(3)
    payload_moment_sum = 1 * aircraft_para.weight.engine * 9.8 * xeg;
    thrust_moment_sum = 1 * thrust * heg;
else
    payload_moment_sum = 0;
    thrust_moment_sum = 0;
end

% Total moment from the tip to the specified position, and torque
moment_sum = weight_moment_sum - thrust_moment_sum + payload_moment_sum - lift_moment_sum;
torque = moment_sum;

end
