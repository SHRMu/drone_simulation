function [x,traj] = multiDroneApproach(x,dis,Kinematic,goal,zoneParam,obs,drones,obsR,periodT)
    [u,traj] = VelocityVectorApproach(x,dis,Kinematic,goal,zoneParam,obs,drones,obsR,periodT);
    % update current status
    x=updateX(x,u,periodT);
end