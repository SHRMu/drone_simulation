function VelocityVectorApproach_single_drone
    close all;
    clear all;
    clc;
    disp('Velocity Vector Program start!!');

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% init %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    num=1;%drone num
    target_num=1;
%     quad_init_x=12*rand(num,1);%init pos
%     quad_init_y=12*rand(num,1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% test %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    quad_init_x=0;%init pos
    quad_init_y=0;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    goal=[
            10 10; 
    %       3 8;
    %       8 3;
    %       8 8
          ];% target [x(m),y(m)]

%     obstacles = [5 6;
%                  5.5 5.5;
%                  6 5;
%                 ];
%     obstacles = [2 4;
%                 2 1;
%                 4 4;
%                 6 5;
%                 ];
%     obstacles = [1 2;
%                1.5 2;
%                2 2;
%                2.5 2;
%                3 2;
%                2 4;
%                2 4.5;
%                2 5;
%                2 5.5;
%                2 6;
%                2.5 6;
%                3 6;
%                6 4;
%                6 4.5;
%                6 5;
%                6 5.5;
%                2 8;
%                2.5 8;
%                3 8;
%                3.5 8;
%                4 8;
%                4 8.5;
%                4 9;
%                4 9.5;
%                4 10;
%                ];
    obstacles = [4 6;
                 4.5 6;
                 5 6;
                 5.3 5.7;
                 5.5 5.5;
                 5.7 5.3;
                 6 5;
                 6 4.5;
                 6 4;
                 ];
%     obstacles = [ 
%                   2 2;
%                   4 6;
%                   4.5 6;
%                   5 6;
%                   5.5 6;
%                   6 6;
%                   6 5.5;
%                   6 5;
%                   6 4.5;
%                   6 4;
%                   6 3.5;
%                   6 3;
%                   ];
                  
    obs = obstacles;
           
    obstacleR=0.1;% parameter r[m]
                  % dimension of the drones

    % velocity params
    Vm = 1.0; %Vmax [m/s]
    Vacc = 0.1; %acc [m/ss]
    Wm = toRadian(20.0);
    Wacc = toRadian(50.0);
    Kinematic = [Vm,Vacc,Wm,Wacc];

    % zone params
    dangerR = 0.8; % paranmeter R
    omega = 2.0;   % parameter omega
    zoneParam = [dangerR,omega*dangerR];

    global dt; dt=0.05;% time[s]
    periodT = 0.1; % every T[s] communication

    area=[-1 12 -1 12];% simulation area [xmin xmax ymin ymax]

    % init status[xi,yi,yaw,v,w]
    x=[quad_init_x(1,1) quad_init_y(1,1) atan2((goal(1,2)-quad_init_y(1,1)),(goal(1,1)-quad_init_x(1,1))) 0 0;]';
    result.x=[]; % save result value
    tic;
    for i=1:5000
        [u,traj,obs,ob] = VelocityVectorApproach(x,Kinematic,goal,zoneParam,obs,obstacleR,periodT);
        % update current status
        x=updateX(x,u);
        % save result
        result.x=[result.x; x'];

        if norm(x(1:2)-goal')<0.5
            disp('Arrive Goal!!');break;
        end

        %====Animation====
        hold off;
        ArrowLength=0.2;
        % drones 
        quiver(x(1,1),x(2,1),ArrowLength*cos(x(3,1)),ArrowLength*sin(x(3,1)),'ok');hold on;
        plotCircle(x(1,1),x(2,1),zoneParam);hold on;
        plot(result.x(:,1),result.x(:,2),'-b');hold on;
        plot(goal(1,1),goal(1,2),'*b');hold on;
        

        plotObstacles(obstacles,dangerR);hold on; 
        if ~isempty(ob)
            plot(ob(:,1),ob(:,2),'d');hold on;
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




