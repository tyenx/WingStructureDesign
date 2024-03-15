function mass = cal_mass_wing(aerofoil_section_geometry, material_properties, stringer_design, spar_design, skin_thickness, rib_distance, D_section_thickness, rib_thickness, spar_thickness_front, spar_thickness_rear)

% Basic data
bref = aerofoil_section_geometry.span; % Section width (spanwise), in meters
b2 = aerofoil_section_geometry.height; % Wing box height (thickness), in meters
length_Dsection = 947.66e-3/2.85; % Perimeter of the D-section skin cross-section, unit (* chord length)
length_wingbox = 2856.69e-3/2.85; % Perimeter of the wing box skin cross-section, unit (* chord length)
length_trailingedge = 3013.14e-3/2.85; % Perimeter of the trailing edge skin cross-section, unit (* chord length)
area = 668267.51e-6/2.85^2; % Wing cross-sectional area, unit (* chord length^2)
rho = material_properties.density; % Material density, in kg/m^3
cr = aerofoil_section_geometry.chord_rootside; % Chord length at the root side, in meters
ct = aerofoil_section_geometry.chord_tipside; % Chord length at the tip side, in meters

% Stringer mass
n_stringer = stringer_design.number; % Number of stringers
ts = stringer_design.thickness; % Stringer thickness, in meters
h = stringer_design.web_height; % Stringer web height, in meters
d = stringer_design.flange_width; % Stringer flange width, in meters
As = ts * (h + 2 * d); % Stringer cross-sectional area, in m^2
mass_stringer = rho * As * bref * n_stringer * 2;

% Spar mass
b_FS = spar_design.flange_width_front; % Front spar flange width, in meters
t1_FS = spar_design.flange_thickness_front; % Front spar flange thickness, in meters
b_RS = spar_design.flange_width_rear; % Rear spar flange width, in meters
t1_RS = spar_design.flange_thickness_rear; % Rear spar flange thickness, in meters
A_FS = 2 * b_FS * t1_FS + (b2 - 2 * t1_FS) * spar_thickness_front; % Front spar cross-sectional area, in m^2
A_RS = 2 * b_RS * t1_RS + (b2 - 2 * t1_RS) * spar_thickness_rear; % Rear spar cross-sectional area, in m^2
mass_spar = rho * (A_FS + A_RS) * bref;

% Rib mass
n_rib = floor(bref / rib_distance); % Number of ribs per section
A_rib_average = area * (cr^2 + ct^2) / 2; % Average rib area, in m^2
mass_rib = rho * A_rib_average * rib_thickness * n_rib;

% Skin mass
A_Dsection = length_Dsection * (cr + ct) * bref / 2; % Surface area of D-section skin (calculated as a trapezoid), in m^2
A_wingbox = length_wingbox * (cr + ct) * bref / 2; % Surface area of wing box skin (calculated as a trapezoid), in m^2
A_trailingedge = length_trailingedge * (cr + ct) * bref / 2; % Surface area of trailing edge skin (calculated as a trapezoid), in m^2
mass_skin = rho * (A_Dsection * D_section_thickness + A_wingbox * skin_thickness + A_trailingedge * skin_thickness);

mass = mass_spar + mass_stringer + mass_rib + mass_skin;

end