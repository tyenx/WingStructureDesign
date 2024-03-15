% 20240215 Calculate aerodynamic force loads on the horizontal tail (actually negative lift) (assuming uniform distribution)
% Input —— flying_state: Flight state quantities, structure
%         aircraft_para: Basic aircraft parameters, structure
%         location: Distance from the root of the wing to the position of interest, scalar, in meters
%         LR: Choose left/right side, string
% Output —— shear: Shear force, scalar, in Newtons
%         bending: Bending moment, scalar, in Newton-meters
%         aero_sum: Sum of aerodynamic forces from the wingtip to the position of interest (i.e., integration), scalar, in Newtons

function [shear, bending, aero_sum] = load_tailH_aero(flying_state, aircraft_para, location, LR)

% Read data
s = aircraft_para.tailscale.span_H / 2; % Half wing span, in meters
x = s - location; % Convert distance from root to distance from tip, in meters
load_case = flying_state.load_case; % Load Case

% Only the second load case involves aerodynamic forces; other cases ignore tail aerodynamics
switch load_case
    case 1
        shear = 0;
        bending = 0;
        aero_sum = 0;
    case 2
        % Reverse calculate unit span aerodynamic force based on total aerodynamic force
        if strcmp(LR, 'left')
            aero_unit = 52e3 * 0.75 / s; % Unit span aerodynamic force, in N/m
        elseif strcmp(LR, 'right')
            aero_unit = 52e3 * 0.25 / s;
        else
            aero_unit = nan;
        end

        % Total aerodynamic force at this position (from tip to this position)
        aero_sum = aero_unit * x;

        % Shear & Bending
        shear = aero_sum;
        bending = 0.5 * aero_unit * x^2;
    case 3
        shear = 0;
        bending = 0;
        aero_sum = 0;
end

end
