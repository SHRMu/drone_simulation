function [u,traj,obs,ob] = VelocityVectorApproach(x,dis,model,goal,zoneParam,obs,drones,R,T) 
    obs = [obs;drones];
    [obs_on,obs_off] = recogObstacles(x,goal,zoneParam,obs);
    ob = [];
    if ~isempty(obs_on)
        ob = obs_on(1,:);
    end
%     obs = [obs;obs_on];
    obs = unique(obs,'rows','stable');
    [vt,ot] = calcVelocity(x,dis,model,goal,zoneParam,obs_on,obs_off,drones,R,T);
    [x,traj]= GenerateTrajectory(x,model,vt,ot,T);
    u = [vt,ot]';
end



