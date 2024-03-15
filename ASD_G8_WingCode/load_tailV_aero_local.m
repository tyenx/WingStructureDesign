% Calculates the local aerodynamic force on the vertical tail for a specific mesh element (assuming uniform distribution)
% Input —— flying_state: Flight state quantities, structure
%         aircraft_para: Basic aircraft parameters, structure
%         location: Distance from the root to the position of interest, scalar, in meters
%         mesh_step: The length of each mesh element on the tail, scalar, in meters
% Output —— aero_local: Local aerodynamic force for the mesh element, scalar, in Newtons

function aero_local = load_tailV_aero_local(flying_state, aircraft_para, location, mesh_step)

% Read data
s = aircraft_para.tailscale.height_V; % Vertical span (height), in meters
x = s - location; % Convert distance from root to distance from tip, in meters
load_case = flying_state.load_case; % Load Case

% Aerodynamic force is only considered in the second load case; it's ignored in others
switch load_case
    case 1
        aero_local = 0;
    case 2
        aero_unit = 34e3 / s; % Unit span aerodynamic force, in N/m
        aero_local = aero_unit * mesh_step; % Local aerodynamic force for the mesh element
    case 3
        aero_local = 0;
end

end
