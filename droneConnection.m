function [obs_dr] = droneConnection(x, goal, zoneParams, drones)
    res = droneInSensorRegion(x,goal,zoneParams,drones);
    [M,N] = size(res.pos);
    if M == 1
        obs_dr = res.pos;
    elseif ~isempty(res.pos)
       obs_dr = sum(res.pos.*res.weight);
    else
        obs_dr = [];
    end
    
end

function res = droneInSensorRegion(x,goal,zoneParams,drones)
    [M,N] = size(drones);
    % only drones before and within the sensor region will be concluded 
    res.pos = [];
    res.dis = [];
    res.weight = [];
    for i = 1:M
        di = norm(drones(i,:)-x(1:2)');
        if di <= zoneParams(3)
            theta2t = angleConversion(rad2deg(atan2(goal(1,2)-x(2),goal(1,1)-x(1)))-rad2deg(x(3)));
            theta2d = angleConversion(rad2deg(atan2(drones(i,2)-x(2),drones(i,1)-x(1)))-rad2deg(x(3)));
            if theta2d > -90 && theta2d < 90
                res.pos = [res.pos;drones(i,:)];
                res.dis = [res.dis;1/di]; % inverse distance weights
            end
        end
    end
    if ~isempty(res.dis)
        sum_dis = sum(res.dis);
        res.weight = res.dis/sum_dis;
    end
    
end
