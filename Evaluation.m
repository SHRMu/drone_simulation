function [evalDB,trajDB] = Evaluation(x,Vt,model,goal,Vr,Ovir,drones,zoneParams,T) 
    evalDB=[];
    trajDB=[];
    for vtx=Vr(1):model(5):Vr(1)
        for vty=Vr(3):model(5):Vr(3)
            for wt=Vr(5):model(6):Vr(5)
                u = [vtx vty wt]';
                [x,traj] = generateTrajectory(x,model,u,T);
%                 heading=CalcHeadingEval(u,Vt);
                dist=CalcDistEval(x,drones,zoneParams(1));
                vel=sqrt(u(1)^2+u(2)^2);
                stopDist=CalcBreakingDist(vel,model);
%                 stopDist = 0;
                if dist>stopDist % 
%                     evalDB=[evalDB;[vtx vty wt heading dist vel]];
                    evalDB=[evalDB;[vtx vty wt vel]];
                    trajDB=[trajDB;traj];
                end
            end
        end
    end
    
end

function heading=CalcHeadingEval(u,Vt)

    theta = rad2deg(atan2(u(2),u(1)));
    Tvtx2Tvty=rad2deg(atan2(Vt(2),Vt(1)));
    
    if Tvtx2Tvty>theta
        targetTheta=Tvtx2Tvty-theta;% [deg]
    else
        targetTheta=theta-Tvtx2Tvty;% [deg]
    end

    heading=180-targetTheta;
end

function dist=CalcDistEval(x,ob,R)
    % ?????????
    dist=100;
    if ~isempty(ob)
        for io=1:length(ob(:,1))
            disttmp=norm(ob(io,:)-x(1:2)')-R;
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