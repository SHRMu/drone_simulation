function [u,traj] = VelocityVectorApproach(x,model,goal,zoneParam,obs,R)
    [vt,ot] = CalcVelocity(x,model,goal,zoneParam,obs,R);
    [x,traj]=  GenerateTrajectory(x,model,vt,ot,0.1);
    u = [vt,ot]';
end

function [vt,ot] = CalcVelocity(x,model,goal,zoneParam,obs,r)
    for io=1:length(obs(:,1))
        di=norm(obs(io,:)-x(1:2)');
        if di < r
            alpha = 0;
            beta = 1;
            gamma = 0;
        elseif di > r && di <= zoneParam(1)
            alpha = 0;
            beta = 0.5*(1+cos(pi*(di-r)/(zoneParam(1)-r)));
            gamma = 0.5*(1-cos(pi*(di-r)/(zoneParam(1)-r)));
        elseif di > zoneParam(1) && di <= zoneParam(2)
            alpha = 0.5*(1-cos(pi*(di-zoneParam(1))/(zoneParam(2)-zoneParam(1))));
            beta = 0;
            gamma = 0.5*(1+cos(pi*(di-zoneParam(1))/(zoneParam(2)-zoneParam(1))));
        elseif di > zoneParam(2)
            alpha = 1;
            beta = 0;
            gamma = 0;
        end
        
        V_t = alpha*model(1);
        theta_t = toDegree(atan2(goal(1,2)-x(2),goal(1,1)-x(1)));
        V_o = beta*model(1);
        theta_o = toDegree(atan2(obs(io,2)-x(2),obs(io,1)-x(1)))-180;
        V_g = gamma*model(1);
        theta_g = toDegree(atan2(obs(io,2)-x(2),obs(io,1)-x(1)))-90;
        
        vtx = V_t*cos(toRadian(theta_t))+V_o*cos(toRadian(theta_o))+V_g*cos(toRadian(theta_g));
        vty = V_t*sin(toRadian(theta_t))+V_o*sin(toRadian(theta_o))+V_g*sin(toRadian(theta_g));
        theta = atan2(vty,vtx);
        
        vt = vtx/cos(theta);
        if theta ~= x(3)
            ot = (theta-x(3))/0.1;
        else
            ot = 0;
        end      
    end
end