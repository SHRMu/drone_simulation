function [x,traj]=GenerateTrajectory(x,model,vt,wt,evaldt)
    global dt;
    time=0;
    u=[vt;wt];% 
    traj=x;% 
    while time<=evaldt
        time=time+dt;% 
        x=updateX(x,u,time);% 
        traj=[traj x];
    end
end