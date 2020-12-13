function VelocityVectorApproach_single
    close all;
    clear all;
    clc;
    disp('Velocity Vector Program start!!');

    %%%%%%%%%%%%%%%%%%%%%%%%%% random %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     quad_init_x=12*rand(num,1);%init pos
%     quad_init_y=12*rand(num,1);

    %%%%%%%%%%%%%%%%%%%%%%%%%%% demo %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    quad_init_x=0;%init pos
    quad_init_y=0;
    goal=[10 10];% target [x(m),y(m)]
    
    %%%%%%%%%%%%%%%%%%%%%%%%% obstacles %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%       obstacles =[
%                   4 4;
%                   4 2;
%                   ];
      
%     obstacles = [
%                 2 4;
%                 3 3;
%                 4 2;
%                 ];

    obstacles = [
               1 2;
               1.5 2;
               2 2;
               2.5 2;
               3 2;
               6 4;
               6 4.5;
               6 5;
               6 5.5;
               6 6;
               6 6.5;
               8 8;
               ];

%     obstacles = [
%                  4 6;
%                  4.5 6;
%                  5 6;
%                  5.3 5.7;
%                  5.5 5.5;
%                  5.7 5.3;
%                  6 5;
%                  6 4.5;
%                  6 4;
%                  6 3.5;
%                  ];

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
              
%      obstacles = [
%                  2 4;
%                  2 4.5;
%                  2 5;
%                  2 5.5;
%                  2 6;
%                  2.5 6;
%                  3 6;
%                  3.5 6;
%                  4 6;
%                  4.5 6;
%                  5 6;
%                  5.3 5.7;
%                  5.5 5.5;
%                  5.7 5.3;
%                  6 5;
%                  6 4.5;
%                  6 4;
%                  6 3.5;
%                  6 3;
%                  6 2.5;
%                  6 2;
%                  5.5 2;
%                  5 2;
%                  4.5 2;
%                  4 2;
% %                  2 2;
%                  ];

%     % random obstacles              
%     random_obs_num = 10;
%     for i = 1:random_obs_num
%         obx = 8*rand(num,1);
%         oby = 8*rand(num,1);
%         obstacles = [obstacles;[obx oby]];
%     end
    run("parameters.m");

    area = [-1 12 -1 12];% simulation area [xmin xmax ymin ymax]

    % init status[xi,yi,yaw,Vx,Vy,W]
    % earth coordinates
    x=[quad_init_x(1,1) quad_init_y(1,1) atan2((goal(1,2)-quad_init_y(1,1)),(goal(1,1)-quad_init_x(1,1))) 0 0 0;]';
    result.quad=x'; % save result value
    result.dis2t = sqrt((goal(1,2)-x(2,1))^2+(goal(1,1)-x(1,1))^2);

    tic;
    
%     writerObj=VideoWriter('simulation.avi');   
%     open(writerObj); 
    obs = linkObstacles(obstacles,0.5); 
    drones = [];
    
    for i=1:5000
        
%         [ut,obs,ob] = VelocityVectorApproach(x,Kinematic,goal,zoneParams,obs,drones,droneR,evalParams);
        [ut,obs,ob] = PotentialFieldApproach(x,Kinematic,goal,zoneParams,obs,drones,droneR,evalParams);
        % update current status
        x=updateX(x,ut);
        % save result
        result.quad=[result.quad; x'];
        result.dis2t=[result.dis2t; sqrt((goal(1,2)-x(2,1))^2+(goal(1,1)-x(1,1))^2)];

        if norm(x(1:2)-goal')< 2*droneR
            disp('Arrive Goal!!');break;
        end

        %====Animation====
        hold off;
        ArrowLength=0.2;
        % drones 
        quiver(x(1,1),x(2,1),ArrowLength*cos(x(3,1)),ArrowLength*sin(x(3,1)),'ok');hold on;
       
        plotCircle(x(1,1),x(2,1),zoneParams);hold on;
        plot(result.quad(:,1),result.quad(:,2),'-b');hold on;
        
        % plot goal
        plot(goal(1,1),goal(1,2),'*r');hold on;
        
        [M,~] = size(obs);
        if M >= 2
            plotObstacles(obs,dangerR);hold on; 
        else
            plot(obs(:,1),obs(:,2),'d');hold on;
        end
       	
        if ~isempty(ob)
            plot(ob(:,1),ob(:,2),'d');hold on;
%         else
%             plot(obs(:,1),obs(:,2),'d');hold on;
        end
        
        % traj
%         if ~isempty(traj)
%             for it=1:length(traj(:,1))/5
%                 ind=1+(it-1)*5;
%                 plot(traj(ind,:),traj(ind+1,:),'-g');hold on;
%             end
%         end

        axis(area);
        grid on;
        drawnow;
        
%         frame = getframe;              
%         writeVideo(writerObj,frame); 
%         close(writerObj); 
    end

    toc;
    
    figure('Name','Distance to Target');
    [N,~] = size(result.dis2t);
    tx = 0:evalParams(4):(N-1)*evalParams(4);
    ty=result.dis2t';
    plot(tx,ty);
    xlabel('t = 0 interval 0.1s');
   
end




