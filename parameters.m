% velocity params
Vm = 1; %Vmax [m/s]
Vacc = 1; %acc [m/ss]
Wm = toRadian(90.0);
Wacc = toRadian(90.0);
Rv = 0.01; % resolution
Rw = 0.01;
Kinematic = [Vm,Vacc,Wm,Wacc,Rv,Rw];

droneR = 0.15;% parameter r[m]
             % dimension of the drones
% zone params
colliR = 0.5; % collision R
dangerR = 1.0; % paranmeter R
omega = 1.5;   % parameter omega
zoneParams = [colliR, dangerR, omega*dangerR];

global dt; dt=0.1;% time[s]
evalParams=[0.5,0.2,0.0,dt];