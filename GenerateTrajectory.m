function [x,traj]=generateTrajectory(x,model,u,evaldt)
    global dt;
    time=0;
%     u=[vtx;vty;wt];% 
    traj=x;% 
    while time<=evaldt
        time=time+dt;% 
        x=updateX(x,u);% 
        traj=[traj x];
    end
end