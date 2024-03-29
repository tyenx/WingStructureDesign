clc
clear
close all

%% Input aircraft design parameters
load Aircraft_Parameter.mat;

%% Provide design parameters and flight state, tentative values
fly_state.load_factor = 1; % Design load factor, unitless
fly_state.load_case = 2;

des_para.xhg_tailV = 0.3; % Distance between rudder hinge and shear center (behind the shear center), in terms of chord length

%%
% The difference between the left and right horizontal tail root moments is the moment applied to the vertical tail
[~, BM_H_left, ~] = load_tailH_aero(fly_state,aircft_para,0,'left');
[~, BM_H_right, ~] = load_tailH_aero(fly_state,aircft_para,0,'right');
BM = BM_H_left-BM_H_right;

%
i = 1;
step = 0.1; % Division of vertical tail increments (longitudinal)
factor = 1; % Plotting factor
for X=0:step:3.5
    % Inertial load
    S_i = 0;
    BM_i = BM;
    Loadi_local = 0;

    % Aerodynamic force
    [S_a, BM_a, ~] = load_tailV_aero(fly_state,aircft_para,X);
    Aero_local = load_tailV_aero_local(fly_state,aircft_para,X,step);

    % Moment (torque)
    [T, ~] = load_tailV_torque(fly_state,aircft_para,des_para,X);
    Moment_local = load_tailV_local_torque(fly_state,aircft_para,des_para,X,step);

    % Record data
    data_shear_i(i,:) = [X,S_i*factor]; % Shear force generated by inertia load
    data_bending_i(i,:) = [X,BM_i*factor]; % Bending moment generated by inertia load
    data_shear_a(i,:) = [X,S_a*factor]; % Shear force generated by aerodynamics
    data_bending_a(i,:) = [X,BM_a*factor]; % Bending moment generated by aerodynamics
    data_shear(i,:) = [X,(S_i+S_a)*factor]; % Shear force
    data_bending(i,:) = [X,(BM_i+BM_a)*factor]; % Bending moment
    data_torque(i,:) = [X,T*factor]; % Torque
    data_loadi_local(i,:) = [X,Loadi_local*factor]; % Local inertia load
    data_aero_local(i,:) = [X,Aero_local*factor]; % Local aerodynamic force
    data_moment_local(i,:) = [X,Moment_local*factor]; % Local external torque
    i = i+1;
end

f1 = figure;
f1.Position = [100 100 800 550];

subplot(3,1,1)
f1p1 = plot(data_loadi_local(:,1),data_loadi_local(:,2));
f1p1_ax = gca;
f1p1_ax.YAxis.Exponent = 3;
title('\rm (a) Inertia Load Distribution')
ylabel('Load (N)');

subplot(3,1,2)
f1p2 = plot(data_shear_i(:,1),data_shear_i(:,2));
f1p2_ax = gca;

title('\rm (b) Shear Force Distribution')
ylabel('Shear Force (N)');

subplot(3,1,3)
f1p3 = plot(data_bending_i(:,1),data_bending_i(:,2));
f1p3_ax = gca;
title('\rm (c) Bending Moment Distribution')
ylabel('Bending Moment (Nm)')
xlabel('VTP Span Stations (m)');

align_Ylabels(gcf)
set([f1p1,f1p2,f1p3],'Color', ...
                'k','LineWidth',1.5)
set([f1p1_ax,f1p2_ax,f1p3_ax],'fontname','CMU Serif', ...
                      'FontSize',14, ...
                      'LineWidth',1, ...
                      'xlim', [0 3.5])

%% Wing Aerodynamic Load 
f2 = figure;
f2.Position = [100 100 800 550];

subplot(3,1,1)
f2p1 = plot(data_aero_local(:,1),real(data_aero_local(:,2)));
f2p1_ax = gca;
f2p1_ax.YAxis.Exponent = 4;
title('\rm (a) Control Surface Force Distribution')
ylabel('Lift (N)');

subplot(3,1,2)
f2p2 = plot(data_shear_a(:,1),data_shear_a(:,2));
f2p2_ax = gca;
title('\rm (b) Shear Force Distribution')
ylabel('Shear Force (N)');

subplot(3,1,3)
f2p3 = plot(data_bending_a(:,1),data_bending_a(:,2));
f2p3_ax = gca;
title('\rm (c) Bending Moment Distribution')
ylabel('Bending Moment (Nm)')
xlabel('VTP Span Stations (m)')

align_Ylabels(gcf)
set([f2p1,f2p2,f2p3],'Color', ...
                'k','LineWidth',1.5)
set([f2p1_ax,f2p2_ax,f2p3_ax],'fontname','CMU Serif', ...
                      'FontSize',14, ...
                      'LineWidth',1, ...
                      'xlim', [0 3.5])

f3 = figure;
f3.Position = [100 100 800 360];

subplot(2,1,1)
f3p1 = plot(data_moment_local(:,1),real(data_moment_local(:,2)));
f3p1_ax = gca;
f3p1_ax.YAxis.Exponent = 3;
title('\rm (a) Wing Local Torque Distribution')
ylabel('Torque (Nm)');

subplot(2,1,2)
f3p2 = plot(data_torque(:,1),data_torque(:,2));
f3p2_ax = gca;
title('\rm (b) Wing Torque Distribution')
ylabel('Torque (Nm)'),
xlabel('VTP Span Stations (m)');

align_Ylabels(gcf)
set([f3p1,f3p2],'Color', ...
                'k','LineWidth',1.5)
set([f3p1_ax,f3p2_ax],'fontname','CMU Serif', ...
                      'FontSize',14, ...
                      'LineWidth',1, ...
                      'xlim', [0 3.5])

f4 = figure;
f4.Position = [100 100 800 700];

subplot(4,1,1)
f4p1 = plot(data_loadi_local(:,1),real(data_loadi_local(:,2)+data_aero_local(:,2)));
f4p1_ax = gca;
title('\rm (a) Total Load Force Distribution')
ylabel('Load (N)')

subplot(4,1,2)
f4p2 = plot(data_shear(:,1),data_shear(:,2));
f4p2_ax = gca;
title('\rm (b) Total Shear Force Distribution')
ylabel('SF (N)'),

subplot(4,1,3)
f4p3 = plot(data_bending(:,1),data_bending(:,2));
f4p3_ax = gca;
title('\rm (c) Total Bending Moment Distribution')
ylabel('BM (Nm)');
ylim('tickaligned')

subplot(4,1,4)
f4p4 = plot(data_torque(:,1),data_torque(:,2));
f4p4_ax = gca;
title('\rm (d) Total Torque Distribution')
ylabel('TQ (Nm)');
xlabel('VTP Span Stations (m)');
ylim('tickaligned')

align_Ylabels(gcf)
set([f4p1,f4p2,f4p3,f4p4],'Color', ...
                'k','LineWidth',1.5)
set([f4p1_ax,f4p2_ax,f4p3_ax,f4p4_ax],'fontname','CMU Serif', ...
                      'FontSize',14, ...
                      'LineWidth',1, ...
                      'xlim', [0 3.5])

% Saving the figures as PDF files
if 1
    exportgraphics(f1,'VTP_inertia_load.pdf','ContentType','vector')
    exportgraphics(f2,'VTP_aero_load.pdf','ContentType','vector')
    exportgraphics(f3,'VTP_torque.pdf','ContentType','vector')
    exportgraphics(f4,'VTP_tot.pdf','ContentType','vector')
end
