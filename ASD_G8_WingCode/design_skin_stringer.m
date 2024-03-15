% 20240225 Skin (thickness) and rib (spacing) design
% Input —— aerofoil_section_geometry: Geometric parameters of the selected section, structure
%         loads_local: Internal forces/moments, structure
%         material_properties: Material mechanical properties, structure
%         stringer_design: Stringer design parameters, structure
%         sigmafig_data: Data for ratio of σ0 to σcr (used for interpolation and lookup), structure
%         FARRAR_data: FARRAR chart data (used for interpolation and lookup), structure
% Output —— t2: Skin thickness, scalar, in meters
%          L: Rib spacing, scalar, in meters

function [t2, L] = design_skin_stringer(aerofoil_section_geometry, loads_local, material_properties, stringer_design, sigmafig_data, FARRAR_data)

% Read data
c = aerofoil_section_geometry.boxwidth; % Wing box width (chordwise), i.e., chord length, in meters
b2 = aerofoil_section_geometry.height; % Wing box height (thickness), in meters
n = stringer_design.number + 1; % Number of regions into which the skin is divided by stringers
ts = stringer_design.thickness; % Stringer thickness, in meters
h = stringer_design.web_height; % Stringer web height, in meters
d = stringer_design.flange_width; % Stringer flange width, in meters
M = loads_local.bending; % Bending moment experienced by the wing, in Nm
E = material_properties.Youngs_modules; % Material Young's modulus, in N/m^2

% Geometric calculation
b = c / n; % Width of a single skin panel (chordwise) divided by stringers, in meters
As = ts * (h + 2 * d); % Stringer cross-sectional area, in m^2

% Calculate skin thickness using buckling equation
N = M / c / b2; % (Chordwise) force per unit length on the skin due to bending (assuming the moment is entirely borne by the skin?), in N/m
t2 = (N / 3.62 / E * b^2)^(1/3); % Calculate skin thickness from buckling equation, in meters; buckling equation for a plate with both ends fixed under normal stress: σ = N / t2 = 3.62 * E * (t2 / b)^2
sigma0 = N / t2; % Normal stress on the skin at this thickness, in N/m^2
% geo_ratio1 = ts / t2; % Two geometric ratio coefficients for lookup
% geo_ratio2 = As / b / t2;
sigma_factor = lookup_sigmafig(sigmafig_data, As, b, t2, ts); % Lookup to get correction factor for skin normal stress
sigmacr = sigma0 * sigma_factor; % Corrected normal stress for skin (assuming stringers are present to share some of the bending moment?), in N/m

% Calculate rib spacing
F = lookup_FARRAR(FARRAR_data, As, b, t2, ts); % Lookup to get FARRAR coefficient
L = (F / sigmacr)^2 * N * E; % Rib spacing (spanwise), in meters
end
