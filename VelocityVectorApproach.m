function [u,traj,obs,ob] = VelocityVectorApproach(x,model,goal,zoneParam,obs,r,T) 
    obs_new = RecogObstacles(x,zoneParam,obs);
    obs_new = unique(obs_new,'rows','stable');
    ob = [];
    if ~isempty(obs_new)
        ob = obs_new(1,:);
    end
    obs = [obs;obs_new];
    obs = unique(obs,'rows','stable');
    [vt,ot] = CalcVelocity(x,model,goal,zoneParam,obs_new,r,T);
    [x,traj]= GenerateTrajectory(x,model,vt,ot,T);
    u = [vt,ot]';
end



