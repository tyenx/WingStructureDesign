clc
clear
close all

%% Input data
% Given aircraft design parameters
load Aircraft_Parameter.mat;

% Flight state
fly_state.air_density = 0.5489; % Air density, in kg/m^3
fly_state.AoA = 0; % Angle of attack, in degrees
fly_state.speed = aircft_para.other.cruise_speed; % Flight speed, in m/s
fly_state.load_factor = 2.5; % Design load factor, unitless

% Basic wing design parameters
des_para.xcg = -(0.4928-0.4); % Distance from center of gravity to shear center (positive in front of the shear center), in terms of chord length
des_para.xac = 0.25-0.1; % Distance from aerodynamic center to shear center (positive in front of the shear center), in terms of chord length
des_para.xeg = 1.8/2.85-0.1; % Distance from engine mounting position to shear center (positive in front of the shear center), in terms of chord length
des_para.heg = 0.6/2.85; % Height difference between engine mounting position and shear center (positive below the shear center), in terms of chord length
des_para.wing_install_AOA = 2.45; % Wing installation angle of attack, in degrees

% Wing root section geometry parameters (NASA SC(2)-0412)
wingbox_width_root = 1.425; % Wing box width (chordwise), i.e., chord length, in meters
wingbox_height_root = 0.276; % Wing box height (thickness), in meters
Dsection_length_root = 0.476; % D-section skin circumferential half perimeter, in meters
Dsection_radius_root = 0.52;  % D-section skin curvature radius, in meters

% Airfoil shape
load Aerofoil_Geometry.mat;

% Material mechanical properties (Al-2024)
material_prop.Youngs_modules = 70e9; % Young's modulus of the material, in N/m^2
material_prop.shear_yield = 187.1e6; % Shear yield strength of the material, in N/m^2
material_prop.compressive_yield = 350e6; % Compressive yield strength of the material, in N/m^2
material_prop.density = 2.78e3; % Density of the material, in kg/m^3

% Figures
load Figure_Data.mat;

% % Temporary
% B = [2592348.472,1709083.642,806636.428,267362.625];
% S = [431096.31,345861.812,226296.622,122042.935];
% To = [150725.985,126364.938,101141.577,73910.274];

%% Design
% Section division
section_length = [2.298,3.22,3.22,5.165]; % Length of each section (spanwise), in meters
section_critical_x = [0,2.298,5.518,8.738,13.903]; % Distance from the wing root to the maximum stress position of each section, in meters
section_chord = [2.85,2.85,2.47,2.1]; % Chord length at the maximum stress position of each section, in meters
section_chord_root = [2.85,2.85,2.47,2.1]; % Chord length on the root side of each section, in meters
section_chord_tip = [2.85,2.47,2.1,1.5]; % Chord length on the tip side of each section, in meters

% Initial design parameters
stringer_thickness = [2,1.8,1.5,1.3]*10^-3; % Thickness of stringers, in meters
stringer_web_height = 50*10^-3; % Height of stringer web, in meters
stringer_flange_web_ratio = 0.3; % Ratio of stringer web height to flange width
% stringer_flange_width = [32,35.2,38.4,44]*1e-3; % Width of stringer flanges, in meters
stringer_number = [18,17,15,12]; % Number of stringers
spar_flange_width_front = [60,50,50,30]*1e-3; % Front spar flange width, in meters
spar_flange_thickness_front = [15,12,12,9]*1e-3; % Front spar flange thickness, in meters
spar_flange_width_rear = [60,50,50,30]*1e-3; % Rear spar flange width, in meters
spar_flange_thickness_rear = [15,12,12,9]*1e-3; % Rear spar flange thickness, in meters

% Design for each section separately
for i = 1:4
    % Calculate loads
    X = section_critical_x(i);
    [S_i, BM_i, ~] = load_wing_inertia(aircft_para,X);
    [S_l, BM_l, ~] = load_wing_lift(fly_state,aircft_para,des_para,X);
    [T, ~] = load_wing_torque(fly_state, aircft_para, des_para, X);
    factor = fly_state.load_factor*aircft_para.other.safefactor; % Load factor * safety factor
    loads.bending = (BM_i+BM_l)*factor;
    loads.shear = (S_i+S_l)*factor;
    loads.torque = T*factor;

    % Calculate geometric dimensions
    wing_section_geom.span = section_length(i);
    wing_section_geom.boxwidth = wingbox_width_root/aircft_para.wingscale.chord_root*section_chord(i);
    wing_section_geom.height = wingbox_height_root/aircft_para.wingscale.chord_root*section_chord(i);
    wing_section_geom.D_length = Dsection_length_root/aircft_para.wingscale.chord_root*section_chord(i);
    wing_section_geom.D_radius = Dsection_radius_root/aircft_para.wingscale.chord_root*section_chord(i);
    wing_section_geom.chord_rootside = section_chord_root(i);
    wing_section_geom.chord_tipside = section_chord_tip(i);

    % Input initial design parameters
    stringer_des.number = stringer_number(i);
    stringer_des.thickness = stringer_thickness(i);
    stringer_des.web_height = stringer_web_height;
    stringer_des.flange_width = stringer_flange_web_ratio*stringer_web_height;
    spar_des.flange_width_front = spar_flange_width_front(i);
    spar_des.flange_thickness_front = spar_flange_thickness_front(i);
    spar_des.flange_width_rear = spar_flange_width_rear(i);
    spar_des.flange_thickness_rear = spar_flange_thickness_rear(i);

    % Skin thickness and rib spacing
    [skin_t,rib_L] = design_skin_stringer(wing_section_geom,loads,material_prop,stringer_des,figure_data_sigmafig,figure_data_FARRAR);
    
    % D-section skin thickness
    D_section_t = design_Dsection(wing_section_geom,material_prop,rib_L,figure_data_ESDU,1e-3);

    % Rib thickness
    rib_t = design_rib(wing_section_geom,loads,material_prop,stringer_des,skin_t,rib_L);

    % Spar thickness & skin thickness correction
    [spar_t_rear,spar_t_front,skin_t_corr] = design_spar_skin(wing_section_geom,loads,material_prop,skin_t,stringer_des,spar_des,figure_data_sigmafig);

    % Calculate center of gravity
    gc = find_gravity_centre(aerofoil_geom_wing,skin_t_corr,D_section_t,spar_t_front,spar_t_rear,0.15,0.65,0.1);

    % Calculate mass
    m = cal_mass_wing(wing_section_geom,material_prop,stringer_des,spar_des,skin_t_corr,rib_L,D_section_t,rib_t,spar_t_front,spar_t_rear);

    % Record data
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
