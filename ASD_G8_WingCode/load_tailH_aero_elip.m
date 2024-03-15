% 20240207 Calculate horizontal tail aerodynamic force loads (actually negative lift) (assuming elliptical distribution) (unused)
% Input —— flying_state: Flight state quantities, structure
%         aircraft_para: Basic aircraft parameters, structure
%         location: Distance from the root of the wing to the position of interest, scalar, in meters
%         LR: Choose left/right side, string
% Output —— shear: Shear force, scalar, in Newtons
%         bending: Bending moment, scalar, in Newton-meters
%         lift_local: Aerodynamic force at the position of interest, scalar, in N/m
%         lift_sum: Sum of aerodynamic forces from the wingtip to the position of interest (i.e., integration), scalar, in Newtons

function [shear, bending, aero_local, aero_sum] = load_tailH_aero_elip(flying_state, aircraft_para, location, LR)

% Read data
s = aircraft_para.tailscale.span_H / 2; % Half wing span, in meters
load_case = flying_state.load_case; % Load Case

% Only the second load case involves aerodynamic forces; other cases ignore tail aerodynamics
switch load_case
    case 1
        shear = 0;
        bending = 0;
        aero_local = 0;
        aero_sum = 0;
    case 2
        % Reverse calculate root aerodynamic force based on total aerodynamic force
        if strcmp(LR, 'left')
            L0 = 52e3 * 0.75 / (s * pi / 4); % Aerodynamic force at the root, in N/m
        elseif strcmp(LR, 'right')
            L0 = 52e3 * 0.25 / (s * pi / 4);
        else
            L0 = nan;
        end
        
        % Lift function and moment function
        daero_fun = @(y) L0 * sqrt(1 - y.^2 / s^2); % Aerodynamic force (lift) function, y is the distance from the root
        dmoment_fun = @(y, x) L0 * sqrt(1 - y.^2 / s^2) .* (y - x); % Moment function due to aerodynamic force, x is the reference point distance from the root
        
        % Aerodynamic force at this position and total aerodynamic force (from tip to this position)
        aero_local = L0 * sqrt(1 - location^2 / s^2);
        aero_sum = integral(daero_fun, location, s);
        
        % Shear & Bending
        shear = aero_sum;
        bending = integral(@(y) dmoment_fun(y, location), location, s);
    case 3
        shear = 0;
        bending = 0;
        aero_local = 0;
        aero_sum = 0;
end

end
