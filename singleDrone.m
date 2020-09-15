function [x(:,n),traj(n,:)] = singleDrone(x,goal,traj,obstacleR,periodT,n,N)
    obstacles = [];
    for i = 1:N
        if i ~= n
            obstacles = [obstacles;x(1,i) x(2,i)];
        end
    end
    if norm(x(1:2,n)-goal(n,:)') > obstacleR
        [x(:,n),traj(n,:)] = multiDroneApproach(x(:,n),Kinematic,goal(n,:),zoneParam,obstacle1,obstacleR,periodT);
    end
end