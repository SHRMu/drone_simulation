function [evalDB] = Evaluation(x,Vt,model,Vr,obs_all,drones,zoneParams) 
    evalDB=[];
    trajDB=[];

    for vtx=Vr(1):model(5):Vr(2)
        for vty=Vr(3):model(6):Vr(4)
            wt = CalcAngular(x,vtx,vty);
            if wt < Vr(5) || wt > Vr(6) || wt*Vt(3) <0
                continue;
            end
            u = [vtx vty wt]';
            heading=CalcHeadingEval(wt,Vt(3));
            dist=CalcDistEval(x,drones,zoneParams(1));
            vel= 1- 0.5*sqrt((u(1)-Vt(1))^2+((u(2)-Vt(2))^2));
            if abs(u(1)*u(2)*Vt(1)*Vt(2)-0) >= 0.01 
                vel = vel - 0.5*abs((u(2)/Vt(2) - u(1)/Vt(1)));
            else
                vel = vel - 0.5*sqrt((u(1)-Vt(1))^2+((u(2)-Vt(2))^2));
            end
            stopDist=CalcBreakingDist(u(1),model);
            if dist>stopDist
                evalDB=[evalDB;[vtx vty wt heading vel]];
            end
        end
    end
    
end

function heading=CalcHeadingEval(wt,Twt)
    Ttheta = rad2deg(wt);
    theta = rad2deg(Twt);

    if Ttheta>theta
        targetTheta=Ttheta-theta;% [deg]
    else
        targetTheta=theta-Ttheta;% [deg]
    end

    heading=180-targetTheta;
end

function dist=CalcDistEval(x,obs,colliR)
    dist=100;
    if ~isempty(obs)
        for io=1:length(obs(:,1))
            disttmp=norm(obs(io,:)-x(1:2)')-colliR;
            if dist>disttmp% 
                dist=disttmp;
            end
        end
    end

end

function stopDist=CalcBreakingDist(vel,model)
    global dt;
    stopDist=0;
    while vel>0
        stopDist=stopDist+vel*dt;
        vel=vel-sqrt(model(2)^2+model(2)^2)*dt;% 
    end
end 