% This function calculates the local aerodynamic force on the horizontal tail (actually negative lift) for a specified load case, assuming uniform distribution.
% Input —— flying_state: Flight state quantities, structure
%         aircraft_para: Basic aircraft parameters, structure
%         location: Distance from the root of the wing to the position of interest, scalar, in meters
%         LR: Choose left/right side, string
%         mesh_step: The discretization step used in the mesh, scalar, in meters
% Output —— aero_local: Local aerodynamic force at the specified position, scalar, in N

function aero_local = load_tailH_aero_local(flying_state, aircraft_para, location, LR, mesh_step)

% Read data
s = aircraft_para.tailscale.span_H / 2; % Half wing span, in meters
x = s - location; % Convert distance from root to distance from tip, in meters
load_case = flying_state.load_case; % Load Case

% Aerodynamic force is considered only in the second load case; it is ignored in other cases
switch load_case
    case 1
        aero_local = 0;
    case 2
        % Calculate unit span aerodynamic force based on total aerodynamic force
        if strcmp(LR, 'left')
            aero_unit = 52e3 * 0.75 / s; % Unit span aerodynamic force, in N/m
        elseif strcmp(LR, 'right')
            aero_unit = 52e3 * 0.25 / s;
        else
            aero_unit = nan;
        end
            
        % Calculate local aerodynamic force based on unit span force and mesh step
        aero_local = aero_unit * mesh_step;
        
    case 3
        aero_local = 0;
end

end
