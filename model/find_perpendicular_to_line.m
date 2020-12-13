function dis = dis2line(goal, obs_1, obs_2)
    dis = abs(det([obs_2-obs_1;goal-obs_1]))/norm(obs_2-obs_1); 
end