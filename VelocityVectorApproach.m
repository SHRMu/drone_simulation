function [u,traj] = VelocityVectorApproach(x,model,goal,zoneParam,obs,R)
    vt = model(1);
    wt = 0;
    [x,traj]=  GenerateTrajectory(x,vt,wt,1.0,model);
    u = [vt,wt]';
end

function Vr=CalcVelocityVr(x,model,goal,obs)
    

end
