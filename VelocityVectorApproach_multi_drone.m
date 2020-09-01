function VelocityVectorApproach_multi_drone
    close all;
    clear all;
    clc;
    disp('Velocity Vector Program start!!');

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% init %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    num=2;%drone num
    target_num=2;
    % quad_init_x=12*rand(num,1);%init pos
    % quad_init_y=12*rand(num,1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% test %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    quad_init_x=[0;5];%init pos
    quad_init_y=[0;5];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    goal=[10 10; 
           0 0;
    %       8 3;
    %       8 8
          ];% target [x(m),y(m)]

%     obstacle = [2 4;
%                 3 3;
%                 4 2;
%                 ];
%     obstacle = [3 2;
%                 2 3;
%                 ];
%     obstacle = [2 4;
%                 2 2;
%                 4 4;
%                 6 5;
%                 ];
    obstacles = [1 2;
               1.5 2;
               2 2;
               2.5 2;
               3 2;
               6 4;
               6 4.5;
               6 5;
               6 5.5;
               ];
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
    x=[quad_init_x(1,1) quad_init_y(1,1) atan2((goal(1,2)-quad_init_y(1,1)),(goal(1,1)-quad_init_x(1,1))) 0 0;
       quad_init_x(2,1) quad_init_y(2,1) atan2((goal(2,2)-quad_init_y(2,1)),(goal(2,1)-quad_init_x(2,1))) 0 0;]';
    result.quad1=[]; % save result value
    result.quad2=[];
    traj1 = [];
    traj2 = [];
    tic;
    for i=1:5000
        
        obstacle1 = [obstacles;x(1,2) x(2,2)];
        obstacle2 = [obstacles;x(1,1) x(2,1)];
        if norm(x(1:2,1)-goal(1,:)') > obstacleR
            [x(:,1),traj1] = multiDroneApproach(x(:,1),Kinematic,goal(1,:),zoneParam,obstacle1,obstacleR,periodT);
        end
        if norm(x(1:2,2)-goal(2,:)') > obstacleR
            [x(:,2),traj2] = multiDroneApproach(x(:,2),Kinematic,goal(2,:),zoneParam,obstacle2,obstacleR,periodT);
        end
        % save result
        result.quad1=[result.quad1; x(:,1)'];
        result.quad2=[result.quad2; x(:,2)'];

        if norm(x(1:2,1)-goal(1,:)') < obstacleR && norm(x(1:2,2)-goal(2,:)') < obstacleR
            disp('Arrive Goal!!');break;
        end

        %====Animation====
        hold off;
        ArrowLength=0.5;
        % drones 
        quiver(x(1,1),x(2,1),ArrowLength*cos(x(3,1)),ArrowLength*sin(x(3,1)),'ok');hold on;
        quiver(x(1,2),x(2,2),ArrowLength*cos(x(3,2)),ArrowLength*sin(x(3,2)),'ok');hold on;
        plotCircle(x(1,1),x(2,1),zoneParam);hold on;
        plotCircle(x(1,2),x(2,2),zoneParam);hold on;
        plot(result.quad1(:,1),result.quad1(:,2),'-b');hold on;
        plot(result.quad2(:,1),result.quad2(:,2),'-m');hold on;
        plot(goal(1,1),goal(1,2),'*b');hold on;
        plot(goal(2,1),goal(2,2),'*m');hold on;
        
        plotObstacles(obstacles,dangerR);hold on; 
%         if ~isempty(ob)
%             plot(ob(:,1),ob(:,2),'d');hold on;
%         end
        % plot danger and sensor zone
        
        
        % traj
        if ~isempty(traj1)
            for it=1:length(traj1(:,1))/5
                ind=1+(it-1)*5;
                plot(traj1(ind,:),traj1(ind+1,:),'-g');hold on;
            end
        end
        if ~isempty(traj2)
            for it=1:length(traj2(:,1))/5
                ind=1+(it-1)*5;
                plot(traj2(ind,:),traj2(ind+1,:),'-m');hold on;
            end
        end
        % DrawQuadrotor(x(1,1),x(2,1));
        axis(area);
        grid on;
        drawnow;
    end
    toc;
end




