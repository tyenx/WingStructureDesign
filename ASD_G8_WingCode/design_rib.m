% 20240225 Rib design
% Input —— aerofoil_section_geometry: Geometric parameters of the selected section, structure
%         loads_local: Internal forces/moments, structure
%         material_properties: Material mechanical properties, structure
%         stringer_design: Stringer design parameters, structure
%         skin_thickness: Skin thickness, scalar, in meters
%         rib_distance: Distance between ribs, scalar, in meters
% Output —— t_r: Rib thickness, scalar, in meters

function t_r = design_rib(aerofoil_section_geometry, loads_internal, material_properties, stringer_design, skin_thickness, rib_distance)

% Read data
c = aerofoil_section_geometry.boxwidth; % Wing box width (chordwise), i.e., chord length, in meters
h_c = aerofoil_section_geometry.height; % Wing box height (thickness), in meters
n = stringer_design.number + 1; % Number of regions into which the skin is divided by stringers
ts = stringer_design.thickness; % Stringer thickness, in meters
h = stringer_design.web_height; % Stringer web height, in meters
d = stringer_design.flange_width; % Stringer flange width, in meters
M = loads_internal.bending; % Bending moment experienced by the wing, in Nm
t2 = skin_thickness; % Wing skin thickness, in meters
L = rib_distance; % Distance between ribs (spanwise), in meters
E = material_properties.Youngs_modules; % Material Young's modulus, in N/m^2
tau_y = material_properties.shear_yield; % Material shear yield strength, in N/m^2

% Geometric calculation
b = c / n; % Width of a single skin panel (chordwise) divided by stringers, in meters
As = ts * (h + 2 * d); % Stringer cross-sectional area, in m^2
t_e = t2 + As / b; % Equivalent skin thickness (averaging stringer area into the skin), in meters
I = (c * t_e)^3 / 12 + c * t_e * (h_c / 2)^2; % Area moment of inertia, in m^4, formula for a rectangular section: I = S^3 / 12 + S * d^2

% Mechanical calculation, determining skin thickness
F = M^2 * L * h_c * t_e * c / 2 / E / I^2; % Force analysis to find the normal force on the rib, in N
t_r_buckling = (F / c / 3.62 / E * h_c^2)^(1/3); % Calculating skin thickness from buckling equation, in meters; equation for a plate with both ends fixed under normal stress: σ = F / (t_r * c) = 3.62 * E * (t_r / h_c)^2
t_r_shear = F / tau_y / c; % Calculating skin thickness from shear strength, in meters; shear stress equation: τ = F / (t_r * c) = τ_yield
t_r = max(t_r_buckling, t_r_shear); % Choose the larger value as the final design value

end