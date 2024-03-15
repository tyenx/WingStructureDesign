clc
clear
close all

%% Input Aircraft Design Parameters
load Aircraft_Parameter.mat;

%% Extract Key Parameters

n_norm = aircft_para.other.nmax_postive; % Maximum positive load factor
n_inv = aircft_para.other.nmax_negative; % Maximum negative load factor
Vc = aircft_para.other.cruise_speed; % Cruise speed
W = aircft_para.weight.MTOW; % Maximum takeoff weight
S = aircft_para.wingscale.area; % Wing area
b = aircft_para.wingscale.span;
CLmin = aircft_para.aerofoil.clmin; % Minimum lift coefficient (negative value)
CLmax = aircft_para.aerofoil.clmax; % Maximum lift coefficient
CLa = aircft_para.aerofoil.dcl_dalfa*57.3; % Lift curve slope
density = 0.5489; % Air density

%%

% Calculate various speeds
VD = 1.4*Vc; % Maximum dive speed (structural speed)
VA = sqrt((2*n_norm*W*9.81)/(density*S*CLmax)); % Maximum maneuvering speed (positive load factor)
VG = sqrt((2*abs(n_inv)*W*9.81)/(density*S*abs(CLmin))); % Maximum maneuvering speed (negative load factor)
Vs_norm = sqrt((2*W*9.81)/(density*S*CLmax)); % Stall speed (positive load factor)

% n-V function (maximum usable load factor)
syms V
n_normV = 0.5*density*S*CLmax*V^2/W/9.81; % Positive load factor, n = L/Wg = 0.5ρV²SCLmax/Wg
n_invV = 0.5*density*S*CLmin*V^2/W/9.81; % Negative load factor, n = L/Wg = 0.5ρV²SCLmin/Wg

% Gust
MGC = S/b; % Mean geometric chord
Mu_g = 2*(W*9.81/S)/(density*MGC*9.81*CLa); % Mass ratio
Kg = 0.88*Mu_g/(5.3+Mu_g); % Correction factor
V_line_20_p = 1+(density*Kg*20*CLa*V)/(2*(W*9.81/S)); % Load factor function at various wind speeds
V_line_20_n = 1+(density*Kg*-20*CLa*V)/(2*(W*9.81/S));
V_line_15_p = @(V) 1+(density*Kg*15.2*CLa*V)/(2*(W*9.81/S));
V_line_15_n = 1+(density*Kg*-15.2*CLa*V)/(2*(W*9.81/S));
V_line_7_p = 1+(density*Kg*7.6*CLa*V)/(2*(W*9.81/S));
V_line_7_n = 1+(density*Kg*-7.6*CLa*V)/(2*(W*9.81/S));

%% Plot
figure
axis([0 250 -2 3.5])
xlabel('Airspeed, V (m/s)')
ylabel('Load Factor, n')
title('V-N Diagram')
grid on
hold on

% Plot VA, VG, VC, VD and upper and lower bounds of n as reference
xline(VG,'--b');
xline(VA,'--b');
xline(Vc,'--b');
xline(VD,'--b');

yline(n_norm,'--b');
yline(n_inv,'--b');

% Mark VA, VG, VC, VD, and Vs
plot(VA,n_norm,'ob','MarkerSize',4,'MarkerFaceColor','y')
text(VA,n_norm,'V_A','FontSize',12,'VerticalAlignment','top','fontname','CMU Serif')
plot(VG,n_inv,'ob','MarkerSize',4,'MarkerFaceColor','y')
text(VG,n_inv,'V_G','FontSize',12,'VerticalAlignment','top','fontname','CMU Serif')
text(VD,-1,'V_D','FontSize',12,'VerticalAlignment','top','fontname','CMU Serif')
text(Vc,-1,'V_C','FontSize',12,'VerticalAlignment','top','fontname','CMU Serif')
plot(Vs_norm,1,'ob','MarkerSize',4,'MarkerFaceColor','y')
text(Vs_norm,1,'V_S+ ','FontSize',12,'HorizontalAlignment','right','fontname','CMU Serif')

%n-V function
fplot(n_normV,[0 VA],'k','LineWidth',2)
fplot(n_invV,[0 VG],'k','LineWidth',2)
n_15 = V_line_15_p(VD);
V_20 = solve(V_line_20_p==2.5,V);
plot([VG,VD],[-1,-1],'k','LineWidth',2)
plot([VA,V_20],[2.5,2.5],'k','LineWidth',2)
plot([V_20,VD],[2.5,n_15],'k','LineWidth',2)
plot([VD,VD],[n_15,-1],'k','LineWidth',2)


%Gust
fplot(V_line_20_p,[0 250],'--r')
fplot(V_line_20_n,[0 250],'--r')
fplot(V_line_15_p,[0 250],'--r')
fplot(V_line_15_n,[0 250],'--r')
fplot(V_line_7_p, [0 250],'--r')
fplot(V_line_7_n, [0 250],'--r')
set(gca,'fontname','CMU Serif','FontSize',14, 'LineWidth',1)