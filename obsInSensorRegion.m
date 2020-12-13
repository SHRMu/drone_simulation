function sensorObs = obsInSensorRegion(x,zoneParams,obs)
    sensorObs = [];
    for oi = 1:length(obs(:,1))
        % distance from obs_oi to x
        di=norm(obs(oi,:)-x(1:2)');
        if di <= zoneParams(3) % within sensor zone
            sensorObs = [sensorObs;obs(oi,1) obs(oi,2)];
        end
    end
end