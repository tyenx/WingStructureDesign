% 20240225 Wing Spar Design and Skin Thickness Correction
% Input:
%     aerofoil_section_geometry: Geometry parameters of the selected section, struct
%     loads_local: Internal forces/internal moments, struct
%     material_properties: Material mechanical properties parameters, struct
%     skin_thickness: Skin thickness, scalar, in meters
%     stringer_design: Stringer design parameters, struct
%     spar_design: Spar design parameters, struct
%     sigmafig_data: Data for σ0 and σcr ratio image (for interpolation and table lookup), struct
% Output:
%     t_RS: Rear spar web thickness, scalar, in meters
%     t_FS: Front spar web thickness, scalar, in meters
%     t_skin: Corrected skin thickness, scalar, in meters

function [t_RS, t_FS, t_skin] = design_spar_skin(aerofoil_section_geometry, loads_local, material_properties, skin_thickness, stringer_design, spar_design, sigmafig_data)

% Read data
% a = wing_section_geometry.span; % Width of the section (spanwise), in meters
c = aerofoil_section_geometry.boxwidth; % Wing box width (chordwise), i.e., chord length, also known as the distance between front and rear spars, in meters
b2 = aerofoil_section_geometry.height; % Wing box height (thickness), i.e., height of the spar, in meters
t2 = skin_thickness; % Wing skin thickness (original design value), in meters
n = stringer_design.number + 1; % Number of regions into which skin is divided by stringers
ts = stringer_design.thickness; % Thickness of stringers, in meters
h = stringer_design.web_height; % Height of stringer web, in meters
d = stringer_design.flange_width; % Width of stringer flange, in meters
b_FS = spar_design.flange_width_front; % Width of front spar flange, in meters
t1_FS = spar_design.flange_thickness_front; % Thickness of front spar flange, in meters
b_RS = spar_design.flange_width_rear; % Width of rear spar flange, in meters
t1_RS = spar_design.flange_thickness_rear; % Thickness of rear spar flange, in meters
M = loads_local.bending; % Bending moment on the wing, in Nm
V = loads_local.shear; % Shear force on the wing, in N
T = loads_local.torque; % Torque on the wing, in Nm
E = material_properties.Youngs_modules; % Young's modulus of the material, in N/m^2
tau_y = material_properties.shear_yield; % Shear yield strength of the material, in N/m^2
sigma_y = material_properties.compressive_yield; % Compressive yield strength of the material, in N/m^2

b1 = c / n; % (Divided by stringers) width (chordwise) of a single skin piece, in meters
% Ks = lookup_ESDU_2(ESDU_data,a,b2); % Look up ESDU table to get coefficient Ks
Ks = 8.1; % Since a/b2 is large, directly set Ks=8.1
As = ts * (h + 2 * d); % Cross-sectional area of stringers, in m^2

% Calculate shear flow
q0 = -T / (2 * c * b2); % Shear flow due to torque, in N/m
q2 = V / (2 * b2); % Shear flow due to shear force, in N/m
q_FS = q2 + q0; % Total shear flow inside the front spar, in N/m
q_RS = q2 - q0; % Total shear flow inside the rear spar, in N/m

% Calculate spar web thicknesses using the (shear stress) flexural equation
t_FS = (abs(q_FS) * b2^2 / (Ks * E))^(1/3); % Front spar web thickness, in meters; Flexural equation for shear stress: τ=q/t=Ks*E*(t/b2)^2
t_RS = (abs(q_RS) * b2^2 / (Ks * E))^(1/3); % Rear spar web thickness, in meters; Flexural equation for shear stress: τ=q/t=Ks*E*(t/b2)^2

% Wing spar area moments
Ixx_FS = 1/6 * b_FS * t1_FS^3 + 1/2 * b_FS * t1_FS * (b2 - t1_FS)^2 + 1/12 * t_FS * (b2 - 2 * t1_FS)^3; % Front spar area moment, in m^4
Ixx_RS = 1/6 * b_RS * t1_RS^3 + 1/2 * b_RS * t1_RS * (b2 - t1_RS)^2 + 1/12 * t_RS * (b2 - 2 * t1_RS)^3; % Rear spar area moment, in m^4

% Correct wing skin thickness based on shear, compressive strength, and flexural equations
t_skin = t2 - 1e-5; % Initially set to original design value - 0.01mm (so that it's the original value for the first iteration)
R = 10;
while (R >= 1 || R_c >= 1 || R_s >= 1)
    t_skin = t_skin + 1e-5; % Solve with precision 0.0in, 1mm

    tau_0 = abs(q0) / t_skin; % Shear stress inside the skin, in N/m^2
    Ixx_skin = 1/6 * c * t_skin^3 + 1/2 * c * t_skin * (b2 - t_skin)^2; % Skin area moment, in m^4
    Ixx = Ixx_skin + Ixx_FS + Ixx_RS; % Total area moment of skin and spars, in m^4
%     sigma_0 = M / b2 / c / t_skin; % Tensile stress inside the skin, in N/m^2
    sigma_0 = M * (b2 / 2) / Ixx; % Tensile stress inside the skin and spars, in N/m^2
    sigma_factor = lookup_sigmafig(sigmafig_data, As, b1, t2, ts); % Look up table to get correction factor for skin tensile stress
    sigma_0 = sigma_0 * sigma_factor; % Corrected tensile stress inside the skin (assuming existence of stringers to share some bending moment?), in N/m

    tau_cr1 = Ks * E * (t_skin / b1)^2; % Maximum shear stress skin can withstand without buckling, in N/m^2; Flexural equation for shear stress: τ=Ks*E*(t2/b1)^2
    tau_cr2 = tau_y; % Shear yield strength of skin material, in N/m^2
    tau_cr = min(tau_cr1, tau_cr2); % Take the smaller value between them as the maximum shear stress the skin can bear
    sigma_cr = sigma_y; % Compressive yield strength of skin material, in N/m^2

    R_c = sigma_0 / sigma_cr;
%     R_c = sigma_factor;
    R_s = tau_0 / tau_cr;
    R = R_s^2 + R_c;
end

end
