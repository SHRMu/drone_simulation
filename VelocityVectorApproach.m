function [u,traj,obs,ob] = VelocityVectorApproach(x,model,goal,zoneParam,obs,r,T) 
    [obs_on,obs_off] = RecogObstacles(x,zoneParam,obs);
    obs_on = unique(obs_on,'rows','stable');
    ob = [];
    if ~isempty(obs_on)
        ob = obs_on(1,:);
    end
    obs = [obs;obs_on];
    obs = unique(obs,'rows','stable');
    [vt,ot] = CalcVelocity(x,model,goal,zoneParam,obs_on,obs_off,r,T);
    [x,traj]= GenerateTrajectory(x,model,vt,ot,T);
    u = [vt,ot]';
end



