function x = f(x, u)
    % Motion Model
    % u = [vt; wt]; current velocity, angular_velocity
    global dt;

    F = [1 0 0 0 0
         0 1 0 0 0
         0 0 1 0 0
         0 0 0 0 0
         0 0 0 0 0];

    B = [dt*cos(x(3)) 0
        dt*sin(x(3)) 0
        0 dt
        1 0
        0 1];

    x= F*x+B*u;
end