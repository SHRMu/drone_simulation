function VelocityVectorApproach_multi_drone
    close all;
    clear all;
    clc;
    disp('Velocity Vector Program start!!');

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% init %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    num=4;%drone num
    target_num=4;
    quad_init_x=12*rand(num,1);%init pos
    quad_init_y=12*rand(num,1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% test %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     quad_init_x=[0;4];%init pos
%     quad_init_y=[0;0];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    goal=[ 8 8; 
           6 6;
          8 3;
          0 0;
          ];% target [x(m),y(m)]

%     obstacle = [2 4;
%                 3 3;
%                 4 2;
%                 ];
%     obstacle = [3 2;
%                 2 3;
%                 ];
%     obstacles = [
%                 2 4;
%                 2 2;
%                 4 4;
%                 6 5;
%                 ];
    obstacles = [
%                1 2;
%                1.5 2;
%                2 2;
%                2.5 2;
%                3 2;
%                6 5.5;
%                6 5;
%                6 4.5;
%                6 4;
%                6.5 4;
%                7 4;
%                7.5 4;
%                8 4;
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
    periodT = 0.05; % every T[s] communication

    area=[-1 12 -1 12];% simulation area [xmin xmax ymin ymax]

    % init status[xi,yi,yaw,v,w]
    x=[quad_init_x(1,1) quad_init_y(1,1) atan2((goal(1,2)-quad_init_y(1,1)),(goal(1,1)-quad_init_x(1,1))) 0 0;
       quad_init_x(2,1) quad_init_y(2,1) atan2((goal(2,2)-quad_init_y(2,1)),(goal(2,1)-quad_init_x(2,1))) 0 0;
       quad_init_x(3,1) quad_init_y(3,1) atan2((goal(3,2)-quad_init_y(3,1)),(goal(3,1)-quad_init_x(3,1))) 0 0;
       quad_init_x(4,1) quad_init_y(4,1) atan2((goal(4,2)-quad_init_y(4,1)),(goal(4,1)-quad_init_x(4,1))) 0 0;]';
    result.quad1=[]; % save result value
    result.quad2=[];
    result.quad3=[]; % save result value
    result.quad4=[];
    traj1 = [];
    traj2 = [];
    traj3 = [];
    traj4 = [];
    tic;
    for i=1:5000
        
        obstacle1 = [obstacles;x(1,2) x(2,2);x(1,3) x(2,3);x(1,4) x(2,4)];
        obstacle2 = [obstacles;x(1,1) x(2,1);x(1,3) x(2,3);x(1,4) x(2,4)];
        obstacle3 = [obstacles;x(1,1) x(2,1);x(1,2) x(2,2);x(1,4) x(2,4)];
        obstacle4 = [obstacles;x(1,1) x(2,1);x(1,2) x(2,2);x(1,3) x(2,3)];
        if norm(x(1:2,1)-goal(1,:)') > obstacleR
            [x(:,1),traj1] = multiDroneApproach(x(:,1),Kinematic,goal(1,:),zoneParam,obstacle1,obstacleR,periodT);
        end
        if norm(x(1:2,2)-goal(2,:)') > obstacleR
            [x(:,2),traj2] = multiDroneApproach(x(:,2),Kinematic,goal(2,:),zoneParam,obstacle2,obstacleR,periodT);
        end
        if norm(x(1:2,3)-goal(3,:)') > obstacleR
            [x(:,3),traj3] = multiDroneApproach(x(:,3),Kinematic,goal(3,:),zoneParam,obstacle3,obstacleR,periodT);
        end
        if norm(x(1:2,4)-goal(4,:)') > obstacleR
            [x(:,4),traj4] = multiDroneApproach(x(:,4),Kinematic,goal(4,:),zoneParam,obstacle4,obstacleR,periodT);
        end
        
        % save result
        result.quad1=[result.quad1; x(:,1)'];
        result.quad2=[result.quad2; x(:,2)'];
        result.quad3=[result.quad1; x(:,3)'];
        result.quad4=[result.quad2; x(:,4)'];

        if norm(x(1:2,1)-goal(1,:)') < obstacleR && norm(x(1:2,2)-goal(2,:)') < obstacleR && norm(x(1:2,3)-goal(3,:)') < obstacleR && norm(x(1:2,4)-goal(4,:)') < obstacleR 
            disp('Arrive Goal!!');break;
        end

        %====Animation====
        hold off;
        ArrowLength=0.5;
        % drones 
        quiver(x(1,1),x(2,1),ArrowLength*cos(x(3,1)),ArrowLength*sin(x(3,1)),'ok');hold on;
        quiver(x(1,2),x(2,2),ArrowLength*cos(x(3,2)),ArrowLength*sin(x(3,2)),'ok');hold on;
        quiver(x(1,3),x(2,3),ArrowLength*cos(x(3,3)),ArrowLength*sin(x(3,3)),'ok');hold on;
        quiver(x(1,4),x(2,4),ArrowLength*cos(x(3,4)),ArrowLength*sin(x(3,4)),'ok');hold on;
        
        plotCircle(x(1,1),x(2,1),zoneParam);hold on;
        plotCircle(x(1,2),x(2,2),zoneParam);hold on;
        plotCircle(x(1,3),x(2,3),zoneParam);hold on;
        plotCircle(x(1,4),x(2,4),zoneParam);hold on;
        
        plot(result.quad1(:,1),result.quad1(:,2),'-b');hold on;
        plot(result.quad2(:,1),result.quad2(:,2),'-m');hold on;
        plot(result.quad3(:,1),result.quad3(:,2),'-r');hold on;
        plot(result.quad4(:,1),result.quad4(:,2),'-g');hold on;
        
        plot(goal(1,1),goal(1,2),'*b');hold on;
        plot(goal(2,1),goal(2,2),'*m');hold on;
        plot(goal(3,1),goal(3,2),'*b');hold on;
        plot(goal(4,1),goal(4,2),'*m');hold on;
        
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
        if ~isempty(traj3)
            for it=1:length(traj3(:,1))/5
                ind=1+(it-1)*5;
                plot(traj3(ind,:),traj3(ind+1,:),'-g');hold on;
            end
        end
        if ~isempty(traj4)
            for it=1:length(traj4(:,1))/5
                ind=1+(it-1)*5;
                plot(traj4(ind,:),traj4(ind+1,:),'-m');hold on;
            end
        end
        % DrawQuadrotor(x(1,1),x(2,1));
        axis(area);
        grid on;
        drawnow;
    end
    toc;
end




