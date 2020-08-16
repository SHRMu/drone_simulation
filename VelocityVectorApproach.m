function [u,traj,obs] = VelocityVectorApproach(x,model,goal,zoneParam,obs,r,T)
    obs_new = RecogObstacles(x,zoneParam,obs);
    obs_new = unique(obs_new,'rows','stable');
    obs = [obs;obs_new];
    obs = unique(obs,'rows','stable');
    [vt,ot] = CalcVelocity(x,model,goal,zoneParam,obs_new,r,T);
    [x,traj]= GenerateTrajectory(x,model,vt,ot,T);
    u = [vt,ot]';
end



