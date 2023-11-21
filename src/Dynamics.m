clear; clc;
close all;

%Select CoG
caso = menu('CoG position','a = L/2-10cm','a = L/2','a = L/2+10cm')      
%Vehicle and tyre DATA
m=1997.6;       %[kg] mass
L = 2.85;       % [m] wheelbase
a_vet =[L/2-0.1 L/2 L/2+0.1];  %[m] front wheelbase
a = a_vet(caso);  
b = L-a ;       %[m] rear wheelbase        
Tf=1.54;        %[m] front track
Tr=1.54;        %[m] rear track
Jz=3728;        %[kg m^2] mass moment of inertia 
g = 9.81;       %[m/s^2]
tau_s = 15;     % steering ratio

%static loads
FzF = m*g*b/L; 
FzR = m*g*a/L;
Perc_F = FzF /(m*g)*100;
Perc_R = 100-Perc_F;
disp(['Static load distribution (%F - %R): ',num2str(round(Perc_F)),'-',num2str(round(Perc_R))])

%Cornering stiffness
load CornStiff_Vs_Fz.mat
% interpolate wheel cornering stiffness versus vertical load
CF_w = interp1(Fz_vet,C_alpha_vet,FzF/2);
CR_w = interp1(Fz_vet,C_alpha_vet,FzR/2);
C_med_w = interp1(Fz_vet,C_alpha_vet,m*g/4);

% axle stiffness
CF = 2*CF_w;    % front
CR = 2*CR_w;    % rear

disp('axle cornering stiffness')
disp(['CF = ',num2str(CF),' N/rad'])
disp(['CR = ',num2str(CR),' N/rad'])
disp(' ')
% check under/over steering
if CR*b-CF*a>0
    disp('>>understeering')
elseif CF*a-CR*b==0
    disp('>>neutral vehicle')
else
    disp('>>oversteering')
    V_cr = sqrt(CF*CR*L^2/(m*(a*CF-b*CR)))*3.6;
    disp(['critical speed: ',num2str(round(V_cr*10)/10),' km/h'])
end
% understeering and slip angle gradients
mf = m*b/L; mr = m-mf;
K_us_an = (mf/CF-mr/CR);
K_beta_an = -mr/CR;
% tangent speed (beta=0)
V_beta0 = sqrt(b*L*CR/a/m)*3.6;
disp(' ')
disp(['understeering gradient: K_US = ',num2str(K_us_an),' rad/(m/s^2)'])
disp(['slip angle gradient: K_beta = ',num2str(K_beta_an),' rad/(m/s^2)'])
disp(['tangent speed: V_beta = ',num2str(V_beta0),' km/h'])


%% default parameters
t_end_sim = 20;
delta_vol_max_ramp = 0;
dvol_max = 0;
t_drift_cs_1 = 0; t_drift_cs_2 = 0;
dvol_drift1 = 0; dvol_drift2 = 0; dvol_drift3 = 0;


sel_man = menu('Manoeuvre:',' Step steer','Ramp steer','Sine Sweep');
if isempty(sel_man)
    sel_man=1;
end
switch sel_man
    case 1
        %         (Step steer)
        t_end_sim = 25;      % [s]
        Vd = 100;                % [km/h]
        dvol_max = input('max steering angle (deg) [default = 10 deg]:');
    case 2
        %        (ramp steer)
        t_end_sim = 25;
        Vd = 40;                % [km/h]
        delta_vol_max_ramp = input('max steering angle (deg) [default = 200 deg]:');
    case 3
        %        (Sine Sweep)
        t_end_sim=45;
        Vd = 100;               % [km/h]
        dvol_max = 20;              % [deg] steering wheel angle
       
end
if isempty(dvol_max)==1
    dvol_max=10; % max steering angle
end
if isempty(delta_vol_max_ramp)==1
    delta_vol_max_ramp = 200; % max steering angle
end
freq=5;

V = input(['choose speed for Simulink manoeuvre (default = ' num2str(Vd) ' km/h): ']);
if isempty(V)
    V=Vd;       % km/h
end
V=V/3.6; 
Vv=V;

% matrix initialization for Simulink linear model 
    A_sim=[(-CF-CR)/(m*Vv),(-CF*a+CR*b-m*Vv^2)/(m*Vv^2);
        (-CF*a+CR*b)/Jz,(-CF*a^2-CR*b^2)/(Jz*Vv)];
    B_sim=[CF/(m*Vv) -CR/(m*Vv);
        (CF*a/Jz) -(CR*b/Jz)];

    C_sim = [1,0
        0,1
        (-CR-CF)/(m*Vv^2),(-CF*a+CR*b)/(m*Vv^3)
        -1, -a/Vv
        -1, b/Vv
        (-CR-CF)/(m),(-CF*a+CR*b)/(m*Vv)];
    D_sim = [0 0;
        0 0;
        CF/(m*Vv^2) CR/(m*Vv^2)
        1 0
        0 1
        CF/m CR/m];

% Run simualtion
dt_sim = 1e-3;          % time step
ay_max = 8;            % [m/s^2] limit acceleration: stop simulation 
beta_max = 10;          % [deg] limit slip angle: stop simulation 


%% run simscape model 
sim('VehicleDynamics.slx');    % run Simulink model

%% POST PROCESSING
% Plot Steering angle

figure('Name','steering angle')
hold all; grid on
plot(delta_steer,'LineWidth',2)
xlabel('time [s]'),
hold on; 
plot(L*ro*180/pi*tau_s,'--k'); 


legend('\delta','\delta_0','Fontsize',18,'location','best')
title('Steering Angle \delta_s')

% Plot beta(slip angle) and psi_dot(yaw rate)

figure('Name','States')
hold all; grid on
subplot(2,1,1)
plot(beta,'LineWidth',2)
xlabel('time [s]')
hold on
plot(b*ro*180/pi,'--k'); 

legend('\beta','\beta_0','Fontsize',16,'location','best')
title('slip angle \beta [deg]','Fontsize',16)

grid on
subplot(2,1,2)
plot(r,'LineWidth',2)
hold on
plot(delta_steer/180*pi,'LineWidth',2),xlabel('time [s]'),

title('\r [deg/s]','Fontsize',16), xlabel('time [s]')
legend('\r [deg/s]','\delta [rad]','Fontsize',16,'location','best')
ylim([-50 50])

% Plot alpha_F e alpha_R (tyre slip angles)
figure('Name','alphaF e R');  hold all
plot(alfaF*180/pi,'LineWidth',2); plot(alfaR*180/pi,'LineWidth',2); 
xlabel('time [s]'); ylabel('\alpha [deg]'); grid on; 
legend('\alpha_F','\alpha_R','Fontsize',16,'location','best')
title('Tyre slip angles')
hold off

figure('Name', 'a_y(t)');
hold all; grid on
plot(a_y,'LineWidth',2)
title('Lateral Acceleration a_y [m/s^2]','Fontsize',16)
xlabel('time [s]')
ylabel('')
legend('a_y [m/s^2]','Fontsize',16,'location','best')
hold off 

 
%% Vehicle trajectory
F_Size=13;
figure('Name','Vehicle CG location');
hold all; grid on
plot(Trajectory.Data(:,1), Trajectory.Data(:,2),'Linewidth',2);
title('CG Trajectory'); 
xlabel('X [m]'); ylabel('Y [m]')
X_G = Trajectory.Data(:,1);
Y_G = Trajectory.Data(:,2);
