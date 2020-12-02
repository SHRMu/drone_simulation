function newx = updateX(x, u)
    % Motion Model
    % [xi,yi,yaw,vt,wt]
    % u = [vtx, vty, wt]; current velocity, angular_velocity
    global dt;

    F = [1 0 0 0 0 0
         0 1 0 0 0 0
         0 0 1 0 0 0
         0 0 0 0 0 0
         0 0 0 0 0 0
         0 0 0 0 0 0];

%     B = [dt*cos(x(3)) -dt*sin(x(3))
%         dt*sin(x(3)) dt*cos(x(3))
%         0 dt
%         1 0
%         0 1];
    B = [dt 0 0
         0 dt 0
         0 0 dt
         1 0 0
         0 1 0
         0 0 1];

    newx= F*x+B*u;
    newx(3) = deg2rad(angleConversion(rad2deg(x(3))));

end