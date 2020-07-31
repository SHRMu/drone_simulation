function [x,traj]=GenerateTrajectory(x,vt,wt,evaldt,model)
    % ??????
    % evaldt???????; 
    global dt;
    time=0;
    u=[vt;wt];% 
    traj=x;% 
    while time<=evaldt
        time=time+dt;% 
        x=f(x,u);% 
        traj=[traj x];
    end
end