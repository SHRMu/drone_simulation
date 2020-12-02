function [Vt] = calcVelocity(x,model,goal,zoneParams,obs_on,obs_off,drones,R,T)

    % no obs within sensor zone
    if isempty(obs_on) && isempty(drones)
%         Tvt = model(1);
        th = angleConversion(toDegree(atan2(goal(1,2)-x(2),goal(1,1)-x(1)))); % theta2target
        Tvtx = model(1)*cos(deg2rad(th));
        Tvty = model(1)*sin(deg2rad(th));
        Twt = (deg2rad(th)-x(3))/T;        
    else
        [Tvtx,Tvty,Twt] = calc(x,model,goal,zoneParams,obs_on,obs_off,drones,R,T);
    end
    
    Vt = [Tvtx,Tvty,Twt]';
    
end

function [Tvtx,Tvty,Twt] = calc(x,model,goal,zoneParams,obs_on,obs_off,drones,r,T)
    vt_mat = [];
    
    if ~isempty(obs_on)
        for io=1:length(obs_on(:,1))
    %         for io=1:1
            di=norm(obs_on(io,:)-x(1:2)');

            theta_t = angleConversion(toDegree(atan2(goal(1,2)-x(2),goal(1,1)-x(1))));
            theta_o = angleConversion(toDegree(atan2(obs_on(io,2)-x(2),obs_on(io,1)-x(1)))-180);

            yaw = angleConversion(toDegree(x(3)));
            delta = abs(angleConversion(theta_t - yaw));

            if di < r
                alpha = 0;
                beta = 1;
                gamma = 0;
            elseif di > r && di <= zoneParams(2)
                alpha = 0;
                beta = 0.5*(1+cos(pi*(di-r)/(zoneParams(2)-r)));
                gamma = 0.5*(1-cos(pi*(di-r)/(zoneParams(2)-r)));
            elseif di > zoneParams(2) && di <= zoneParams(3)
                %
                if abs(angleConversion(theta_t - theta_o)) < 90
                    alpha = 1;
                    beta = 0;
                    gamma = 0;
                else
                    alpha = 0.5*(1-cos(pi*(di-zoneParams(2))/(zoneParams(3)-zoneParams(2))));
                    beta = 0;
                    gamma = 0.5*(1+cos(pi*(di-zoneParams(2))/(zoneParams(3)-zoneParams(2))));
                end
            else
                alpha = 1;
                beta = 0;
                gamma = 0;
            end

            V_t = alpha*model(1);
            V_o = beta*model(1);
            V_g = gamma*model(1);
            
%             if abs(angleConversion(theta_o - 90)-theta_t) < 90
%                 theta_g = angleConversion(theta_o - 90);
%             else
%                 theta_g = angleConversion(theta_o + 90);
%             end
            theta_g = angleConversion(theta_o + 90);

%             if abs(angleConversion(theta_g - yaw)) > 90
%                 theta_g = angleConversion(theta_g + 180);
%             end

            vtx = V_t*cos(toRadian(theta_t))+V_o*cos(toRadian(theta_o))+V_g*cos(toRadian(theta_g));
            vty = V_t*sin(toRadian(theta_t))+V_o*sin(toRadian(theta_o))+V_g*sin(toRadian(theta_g));
            
            vt_mat = [vt_mat;vtx vty];
        end
    else
        V_t = model(1);
        theta_t = angleConversion(toDegree(atan2(goal(1,2)-x(2),goal(1,1)-x(1))));
        vtx = V_t*cos(toRadian(theta_t));
        vty = V_t*sin(toRadian(theta_t));
        vt_mat = [vt_mat;vtx vty];   
    end
    
%     if ~isempty(drones)
%         [M,~] = size(drones);
%         obs_on = [];
%         for i = 1:M
%             Vangle = angleConversion(toDegree(atan2(vty,vtx)));
%             Dangle = angleConversion(toDegree(atan2(drones(i,2)-x(2),drones(i,1)-x(1))));
%             theta = angleConversion(Vangle-Dangle);
%             if abs(theta) < 90
% %                 obs_on = [obs_on;drones(i,:)];
%                 vt_mat = [0 0];
%             end
%         end
%     end
    
    [M,~] = size(vt_mat);
    if M > 1
        vt_mat = mean(vt_mat);
    end
    
    Tvtx = vt_mat(1);
    Tvty = vt_mat(2);
%     Tvt = sqrt(vt_mat(1)^2 + vt_mat(2)^2);
    th = atan2(vt_mat(2),vt_mat(1));
    Twt = (th-x(3))/T;
end


