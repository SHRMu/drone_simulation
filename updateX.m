function newx = updateX(x, u, T)
    % Motion Model
    % [xi,yi,yaw,v,w,dis]
    % u = [vt; wt]; current velocity, angular_velocity
    global dt;

    F = [1 0 0 0 0 0
         0 1 0 0 0 0
         0 0 1 0 0 0
         0 0 0 0 0 0
         0 0 0 0 0 0
         0 0 0 0 0 1];

    B = [dt*cos(x(3)) 0
        dt*sin(x(3)) 0
        0 dt
        1 0
        0 1
        T 0];

    newx= F*x+B*u;
end