% 20240229 Calculate local wing lift loads (assuming elliptical distribution)
% Input —— flying_state: Flight state quantities, structure
%         aircraft_para: Basic aircraft parameters, structure
%         location: Distance from the root of the wing to the position of interest, scalar, in meters
%         mesh_step: The length of the wing element divided, scalar, in meters
% Output —— lift_local: Equivalent lift at the position of interest, scalar, in Newtons

function lift_local = load_wing_lift_local(flying_state, aircraft_para, design_para, location, mesh_step)

% Read data
s = (aircraft_para.wingscale.span - aircraft_para.fuslagescale.cabin) / 2; % Half wing span (excluding fuselage width)
AOA = flying_state.AoA + design_para.wing_install_AOA; % Angle of attack, in degrees
L0 = 0.5 * flying_state.air_density * flying_state.speed^2 * aircraft_para.wingscale.chord_root * (aircraft_para.aerofoil.cl0 + AOA * aircraft_para.aerofoil.dcl_dalfa); % Lift at the wing root, in N/m

% Lift at this position (linear approximation)
lift_local = L0 * (sqrt(1 - location^2 / s^2) + sqrt(1 - (location + mesh_step)^2 / s^2)) / 2;

end
