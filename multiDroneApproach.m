function [x,traj] = multiDroneApproach(x,Kinematic,goal,zoneParam,obstacle1,obstacleR,periodT)
    [u,traj] = VelocityVectorApproach(x,Kinematic,goal,zoneParam,obstacle1,obstacleR,periodT);
    % update current status
    x=updateX(x,u);
end