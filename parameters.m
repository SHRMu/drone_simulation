% velocity params
Vm = 1; %Vmax [m/s]
Vacc = 1; %Vacc [m/ss]
Wm = deg2rad(180.0); %Wmax [rad/s]
Wacc = deg2rad(180.0); %Wacc [rad/ss]
Rv = 0.01; % resolution
Rw = 0.01;
Kinematic = [Vm,Vacc,Wm,Wacc,Rv,Rw];

droneR = 0.15;% parameter r[m]
              % dimension of the drones

%%%%% single drone
% colliR = 0.15; % margin R
% dangerR = 0.8;  % danger R
% omega = 2;    % parameter omega
% zoneParams = [colliR, dangerR, omega*dangerR];

%%%%% multi agent params
colliR = 1.0; % margin R
dangerR = 1.5; % paranmeter R
omega = 3;   % parameter omega
zoneParams = [colliR, dangerR, omega*dangerR];

global dt; dt=0.1;% time[s]
evalParams=[0.5,0.1,0.0,dt];