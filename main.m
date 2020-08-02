function main
    close all;
    clear all;
    clc;
    disp('Velocity Vector Program start!!');

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% init %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    num=1;%drone num
    target_num=1;
    % quad_init_x=12*rand(num,1);%init pos
    % quad_init_y=12*rand(num,1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% test %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    quad_init_x=0;%init pos
    quad_init_y=0;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    goal=[10 10; 
    %       3 8;
    %       8 3;
    %       8 8
          ];% target [x(m),y(m)]

%     obstacle = [2 4;
%                 2 2;
%                 4 4;
%                 6 5;
%                 ];
    obstacle = [2 2;
               2.5 2;
               3 2;
               3.5 2;
               6 4;
               6 4.5;
               6 5;
               6 5.5;
               ];
    obstacleR=0.1;% parameter r

    % velocity params
    Vm = 1.0; %Vmax [m/s]
    acc = 0.1; %acc [m/ss]
    Kinematic = [Vm,acc];

    % zone params
    dangerR = 0.5;
    omega = 2.0;
    zoneParam = [dangerR,omega*dangerR];

    global dt; dt=0.1;% time[s]

    area=[-1 12 -1 12];% simulation [xmin xmax ymin ymax]

    % init status[xi,yi,yaw,v,w]
    x=[quad_init_x(1,1) quad_init_y(1,1) atan2((goal(1,2)-quad_init_y(1,1)),(goal(1,1)-quad_init_x(1,1))) 0 0;]';
    result.x=[];
    tic;
    for i=1:5000
        [u,traj] = VelocityVectorApproach(x,Kinematic,goal,zoneParam,obstacle,obstacleR);
        % update current status
        x=f(x,u);
        % save result
        result.x=[result.x; x'];

        if norm(x(1:2)-goal')<0.5
            disp('Arrive Goal!!');break;
        end

        %====Animation====
        hold off;
        ArrowLength=0.5;
        % drones 
        quiver(x(1,1),x(2,1),ArrowLength*cos(x(3,1)),ArrowLength*sin(x(3,1)),'ok');hold on;
        plot(result.x(:,1),result.x(:,2),'-b');hold on;
        plot(goal(1,1),goal(1,2),'*b');hold on;
        plot(obstacle(:,1),obstacle(:,2),'d');hold on;
        % plot danger and sensor zone
        for io=1:length(obstacle(:,1))
            plotCircle(obstacle(io,1),obstacle(io,2),zoneParam);hold on;
        end
        
        % traj
        if ~isempty(traj)
            for it=1:length(traj(:,1))/5
                ind=1+(it-1)*5;
                plot(traj(ind,:),traj(ind+1,:),'-g');hold on;
            end
        end
        % DrawQuadrotor(x(1,1),x(2,1));
        axis(area);
        grid on;
        drawnow;
    end
    toc;
end




