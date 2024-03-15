%20240308 Vertical Tail Design
clc
clear
close all

%% Input Data
% Given aircraft design parameters
load Aircraft_Parameter.mat;

% Flight state
fly_state.air_density = 0.5489; % Air density, unit: kg/m^3
fly_state.AoA = 0; % Angle of attack, unit: degrees
fly_state.speed = aircft_para.other.cruise_speed; % Flight speed, unit: m/s
fly_state.load_factor = 2.5; % Design load factor, unit: -
fly_state.load_case = 2;

% Basic design parameters for the vertical tail
des_para.xhg_tailV = -(0.7-0.4); % Distance between elevator hinge and shear center (positive in front of the shear center), unit: *chord length

% Geometric parameters of the wing root section (NACA-0009 9% smoothed)
wingbox_width_root = 2.541; % Wingbox width (chordwise), i.e., chord length, unit: m
wingbox_height_root = 0.416; % Wingbox height (thickness), unit: m
Dsection_length_root = 0.772; % D-section skin cross-sectional perimeter, unit: m
Dsection_radius_root = 0.843; % D-section skin cross-sectional radius of curvature, unit: m

% Airfoil shape
load Aerofoil_Geometry.mat;

% Material mechanical properties (Al-2024)
material_prop.Youngs_modules = 70e9; % Material Young's modulus, unit: N/m^2
material_prop.shear_yield = 187.1e6; % Material shear yield strength, unit: N/m^2
material_prop.compressive_yield = 350e6; % Material compressive yield strength, unit: N/m^2
material_prop.density = 2.78e3; % Material density, unit: kg/m^3

% Figures
load Figure_Data.mat;

%% Tail Design
% Section division
section_length = [1.75,1.75]; % Length of each section (spanwise), unit: m
section_critical_x = [0,1.75]; % Distance from wing root to each section's maximum stress position, unit: m
section_chord = [4.62,3.511]; % Chord length at each section's maximum stress position, unit: m
section_chord_root = [4.62,3.511]; % Chord length near the wing root side for each section, unit: m
section_chord_tip = [3.511,2.402]; % Chord length near the wing tip side for each section, unit: m

% Initial design parameters
stringer_thickness = 3e-3; % Stringer thickness, unit: m
stringer_web_height = 35e-3; % Stringer web height, unit: m
stringer_flange_web_ratio = [0.3,0.3]; % Stringer web height to flange width ratio
stringer_number = [12,11]; % Number of stringers
spar_flange_width_front = [30,30]*1e-3; % Front spar flange width, unit: m
spar_flange_thickness_front = 15e-3; % Front spar flange thickness, unit: m
spar_flange_width_rear = [30,30]*1e-3; % Rear spar flange width, unit: m
spar_flange_thickness_rear = 15e-3; % Rear spar flange thickness, unit: m

% Design for each section separately
for i = 1:2
    % Calculate loads
    X = section_critical_x(i);
    [S_l, BM_l, ~] = load_tailV_aero(fly_state,aircft_para,X);
    [T, ~] = load_tailV_torque(fly_state,aircft_para,des_para,X);
    factor = fly_state.load_factor*aircft_para.other.safefactor; % Load factor * safety factor
    loads.bending = BM_l*factor;
    loads.shear = S_l*factor;
    loads.torque = T*factor;

    % Calculate geometric dimensions
    wing_section_geom.span = section_length(i);
    wing_section_geom.boxwidth = wingbox_width_root/aircft_para.tailscale.chord_root_V*section_chord(i);
    wing_section_geom.height = wingbox_height_root/aircft_para.tailscale.chord_root_V*section_chord(i);
    wing_section_geom.D_length = Dsection_length_root/aircft_para.tailscale.chord_root_V*section_chord(i);
    wing_section_geom.D_radius = Dsection_radius_root/aircft_para.tailscale.chord_root_V*section_chord(i);
    wing_section_geom.chord_rootside = section_chord_root(i);
    wing_section_geom.chord_tipside = section_chord_tip(i);

    % Input initial design parameters
    stringer_des.number = stringer_number(i);
    stringer_des.thickness = stringer_thickness;
    stringer_des.web_height = stringer_web_height;
    stringer_des.flange_width = stringer_flange_web_ratio(i)*stringer_web_height;
    spar_des.flange_width_front = spar_flange_width_front(i);
    spar_des.flange_thickness_front = spar_flange_thickness_front;
    spar_des.flange_width_rear = spar_flange_width_rear(i);
    spar_des.flange_thickness_rear = spar_flange_thickness_rear;

    % Skin thickness and rib spacing
    [skin_t,rib_L] = design_skin_stringer(wing_section_geom,loads,material_prop,stringer_des,figure_data_sigmafig,figure_data_FARRAR);
    
    % D-section skin thickness
    D_section_t = design_Dsection(wing_section_geom,material_prop,rib_L,figure_data_ESDU,1e-3);

    % Rib thickness
    rib_t = design_rib(wing_section_geom,loads,material_prop,stringer_des,skin_t,rib_L);

    % Spar thickness & skin thickness correction
    [spar_t_rear,spar_t_front,skin_t_corr] = design_spar_skin(wing_section_geom,loads,material_prop,skin_t,stringer_des,spar_des,figure_data_sigmafig);

    % Calculate center of gravity
    gc = find_gravity_centre(aerofoil_geom_tail,skin_t_corr,D_section_t,spar_t_front,spar_t_rear,0.15,0.7,0.09);

    % Calculate mass
    m = cal_mass_tail(wing_section_geom,material_prop,stringer_des,spar_des,skin_t_corr,rib_L,D_section_t,rib_t,spar_t_front,spar_t_rear);

    % Data summary recording
    data_bending(i) = loads.bending;
    data_shear(i) = loads.shear;
    data_torque(i) = loads.torque;
    data_skin_t(i) = skin_t*1e3;
    data_rib_L(i) = rib_L;
    data_D_section_t(i) = D_section_t*1e3;
    data_rib_t(i) = rib_t*1e3;
    data_spar_t_rear(i) = spar_t_rear*1e3;
    data_spar_t_front(i) = spar_t_front*1e3;
    data_skin_t_corr(i) = skin_t_corr*1e3;
    data_gc(:,i) = gc;
    data_m(:,i) = m;
end
