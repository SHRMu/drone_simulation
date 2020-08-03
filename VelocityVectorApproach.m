function [u,traj] = VelocityVectorApproach(x,model,goal,zoneParam,obs,r,T)
    obs_new = RecogObstacles(x,zoneParam,obs);
    [vt,ot] = CalcVelocity(x,model,goal,zoneParam,obs_new,r,T);
    [x,traj]= GenerateTrajectory(x,model,vt,ot,T);
    u = [vt,ot]';
end



