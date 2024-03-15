% 20240225 D-section design
% Input —— aerofoil_section_geometry: Geometric parameters of the selected section, structure
%         material_properties: Material mechanical properties, structure
%         rib_distance: Distance between ribs, scalar, in meters
%         ESDU_data: ESDU chart data (for interpolation and lookup), structure
%         t0: Starting point for iteration, scalar, in meters
% Output —— t1: D-section skin thickness, scalar, in meters

function t1 = design_Dsection(aerofoil_section_geometry, material_properties, rib_distance, ESDU_data, t0)

% Read data
bref = aerofoil_section_geometry.span; % Section width (spanwise), in meters
b = aerofoil_section_geometry.D_length; % Perimeter of the D-section skin cross-section, in meters
R1 = aerofoil_section_geometry.D_radius; % Radius of curvature for the D-section skin, in meters
L = rib_distance; % Distance between ribs, in meters
E = material_properties.Youngs_modules; % Material Young's modulus, in N/m^2
tau_y = material_properties.shear_yield; % Material shear yield strength, in N/m^2

n_rib = floor(bref / L); % Number of ribs per section
a = bref / (n_rib + 1); % Width of a single skin panel (spanwise) divided by ribs, in meters

% Iterate to find skin thickness
t_begin = -1;
t_end = t0; % Set initial skin thickness to the starting value
while(abs(t_begin - t_end) > 1e-5) % Solve to a precision of 0.01 mm
    t_begin = t_end;
    Ks = lookup_ESDU(ESDU_data, a, b, R1, t_begin); % Lookup in ESDU chart to get coefficient Ks
    t_end = sqrt(tau_y * b^2 / Ks / E); % D-Section skin thickness, in meters; assuming the skin is subject to the ultimate shear stress (this ensures failure due only to exceeding shear strength and not due to buckling), shear stress warping equation: τ = Ks * E * (t / b)^2
end
t1 = t_end;

end