function [vt,wt] = CalcVelocity(x,model,goal,zoneParam,obs_on,obs_off,r,T)
    vt_mat = [];
    % no obs within sensor zone
    if isempty(obs_on)
        V_t = model(1);
        theta_t = AngleConversion(toDegree(atan2(goal(1,2)-x(2),goal(1,1)-x(1))));
        vtx = V_t*cos(toRadian(theta_t));
        vty = V_t*sin(toRadian(theta_t));
        vt_mat = [vt_mat;vtx vty];
    else
        for io=1:length(obs_on(:,1))
            di=norm(obs_on(io,:)-x(1:2)');
            delta = AngleConversion(toDegree(atan2(goal(1,2)-x(2),goal(1,1)-x(1)))) - AngleConversion(toDegree(x(3)));
            delta = abs(delta);
            if di < r
                alpha = 0;
                beta = 1;
                gamma = 0;
            elseif di > r && di <= zoneParam(1)
                alpha = 0;
                beta = 0.5*(1+cos(pi*(di-r)/(zoneParam(1)-r)));
                gamma = 0.5*(1-cos(pi*(di-r)/(zoneParam(1)-r)));
            elseif di > zoneParam(1) && di <= zoneParam(2)
                if delta < 90
                    alpha = 0.5*(1-cos(pi*(di-zoneParam(1))/(zoneParam(2)-zoneParam(1))));
                    beta = 0;
                    gamma = 0.5*(1+cos(pi*(di-zoneParam(1))/(zoneParam(2)-zoneParam(1))));
                else 
                    alpha = 0;
                    beta = 0;
                    gamma = 1;
                end 
            end
        
            V_t = alpha*model(1);
            theta_t = AngleConversion(toDegree(atan2(goal(1,2)-x(2),goal(1,1)-x(1))));
            V_o = beta*model(1);
            theta_o = AngleConversion(toDegree(atan2(obs_on(io,2)-x(2),obs_on(io,1)-x(1)))-180);
            V_g = gamma*model(1);
            theta_g = theta_o - 90;
%             theta_g = AngleConversion(toDegree(atan2(obs(io,2)-x(2),obs(io,1)-x(1)))-90);
            if abs(theta_t - theta_g) > 90
                theta_g = AngleConversion(theta_g + 180);
            end
        
            vtx = V_t*cos(toRadian(theta_t))+V_o*cos(toRadian(theta_o))+V_g*cos(toRadian(theta_g));
            vty = V_t*sin(toRadian(theta_t))+V_o*sin(toRadian(theta_o))+V_g*sin(toRadian(theta_g));
            vt_mat = [vt_mat;vtx vty];
        end
        [M,~] = size(vt_mat);
        if M > 1
            vt_mat = mean(vt_mat);
        end
        
%         if ~isempty(obs_off)
%             theta2o = toDegree(atan2(obs_off(1,2)-x(2),obs_off(1,1)-x(1)));
%             if theta2o < 90
%                 vt_tmp = [vt_mat(2),vt_mat(1)];
%                 vt_mat = vt_tmp;
%             end
%         end

    end
    % last T param
    Otheta = x(3);
    Ovtx = cos(Otheta)*x(4);
    Ovty = sin(Otheta)*x(4);
    Owt = x(5); 
    
    % now T
    vtx = vt_mat(1);
    vty = vt_mat(2);
    
    if abs(vtx-Ovtx)/T > model(2)
        vtx = Ovtx + (vtx-Ovtx)/abs(vtx-Ovtx)*model(2);
    end
    if abs(vty-Ovty)/T > model(2)
        vty = Ovty + (vty-Ovty)/abs(vty-Ovty)*model(2);
    end
    
    theta = atan2(vty,vtx);
    vt = vtx/cos(theta);
    
    if theta ~= x(3)
        wt = (theta-x(3))/T;
        if abs(wt-Owt)/T > model(4)
            wt = Owt + (wt-Owt)/abs(wt-Owt)*model(4);
    	end 
    else
        wt = 0;
    end 
end