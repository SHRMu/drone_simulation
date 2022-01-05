function VelocityVectorApproach_multi_main
    close all;
    clear all;
    clc;
    format short;
    disp('Velocity Vector Program - multi drones test start!!');

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% init %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    num = 4;%drone num
%     scale = 3*num;
    scale = 1.5*num;
    radius = 1*scale;
%     area=[-0.5*scale 2.5*scale -0.5*scale 2.5*scale];% simulation area [xmin xmax ymin ymax]
    
    
    area = [-2,14,-2, 14];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% random %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%     quad_init_x=1.5*scale*rand(num,1);%init pos
%     quad_init_y=1.5*scale*rand(num,1);
%     goal = 2.5*scale*rand(num,2);
% %     
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% circle %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    centre = radius*ones(1,2);
    theta = 360/num;
    quad_init_x = zeros(num,1);
    quad_init_y = zeros(num,1);
    goal = zeros(num,2);
    
    for i = 1:num
        quad_init_x(i,1) = centre(1,1) + cos(deg2rad(i*theta))*radius;
        quad_init_y(i,1) = centre(1,2) + sin(deg2rad(i*theta))*radius;
        goal(i,1:2) = [centre(1,1) + cos(deg2rad(i*theta+180))*radius centre(1,1) + sin(deg2rad(i*theta+180))*radius];
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% line %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     quad_init_x = zeros(num,1);
%     quad_init_y = zeros(num,1);
%     goal = zeros(num,2);
%     line_num = num/2;
%     interval = 4;
%     total = (line_num-1)*interval;
%     for i = 1:line_num
%         quad_init_x(i) = 0.1*scale*(total/2 - interval*(i-1))+20;
%         quad_init_y(i) = 0.1*scale*interval+20;
%     end
%     for i = line_num+1:num
%         quad_init_x(i) = 0.1*scale*(-total/2 + interval*(i-line_num-1))+20;
%         quad_init_y(i) = -0.1*scale*interval+20;
%     end
%     for i = 1:num
%         if i <= line_num
%             goal(i,1:2) = [quad_init_x(i+line_num) quad_init_y(i+line_num)];
%         else
%             goal(i,1:2) = [quad_init_x(i-line_num) quad_init_y(i-line_num)];
%         end
%     end
%    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%     load("demo.mat");
    run("parameters.m");

    obstacles = [

               ];
           
    if ~isempty(obstacles)
        obs = linkObstacles(obstacles,0.5);
    else
        obs = [];
    end

    % init status[xi,yi,yaw,Vx,Vy,W]
    x = zeros(num,6);
    for i = 1:num
        x(i,:) = [quad_init_x(i,1) quad_init_y(i,1) atan2((goal(i,2)-quad_init_y(i,1)),(goal(i,1)-quad_init_x(i,1))) 0 0 0];
    end
    x = x';
   
    drones=x(1:2,1:num)';
    
    for di = 1:num
        results(di).obsi = [];
        results(di).quad = []; 
        results(di).traj = [];
        results(di).dis2t = sqrt((goal(di,2)-x(2,di))^2+(goal(di,1)-x(1,di))^2);
        results(di).dis2d = []; % nearest neib drone
    end
    
    % arrived drone -> 1
    arrived = zeros(num,1);

    tic;
    
%     writerObj=VideoWriter('simulation.avi');   
%     open(writerObj); 
    
    for i=1:5000

        for di = 1:num
            if norm(x(1:2,di)-goal(di,:)') > 2*droneR 
                obsi = drones;
                obsi(di,:) = [];
                results(di).obsi = obsi;
                [u,obs,ob] = VelocityVectorApproach(x(:,di),Kinematic,goal(di,:),zoneParams,obs,results(di).obsi,droneR,evalParams);
                % update current status
                x(:,di)=updateX(x(:,di),u);
                drones=x(1:2,1:num)';
            else
                obs = [obs;x(1,di) x(2,di)];
                arrived(di,1) = 1;
            end
            results(di).dis2t=[results(di).dis2t; sqrt((goal(di,2)-x(2,di))^2+(goal(di,1)-x(1,di))^2)];
            results(di).dis2d=[results(di).dis2d; nearestNeighbour(x(:,di),results(di).obsi)];
            results(di).quad=[results(di).quad; x(:,di)'];
        end
        
%         drones=x(1:2,1:num)';

        if all(arrived) == 1
            disp('Arrive Goal!!');break;
        end

        %====Animation====
        hold off;
        ArrowLength=0.2;
        
        for di = 1:num
            quiver(x(1,di),x(2,di),ArrowLength*cos(x(3,di)),ArrowLength*sin(x(3,di)),'ok');hold on;
%             plotCircle(x(1,di),x(2,di),zoneParams);hold on;
            plot(results(di).quad(:,1),results(di).quad(:,2));hold on;
            plot(goal(di,1),goal(di,2),'*');hold on;
        end
        
%         plotObstacles(obstacles,dangerR);hold on; 
%         if ~isempty(ob)
%             plot(ob(:,1),ob(:,2),'d');hold on;
%         end
        
        % DrawQuadrotor(x(1,1),x(2,1));
        axis(area);
        xlabel('X(m)');
        ylabel('Y(m)');
        grid on;
        drawnow;
%         
%         frame = getframe;              
%         writeVideo(writerObj,frame); 
        
    end
    %         close(writerObj); 
    toc;
    
    figure('Name','Distance to Target');
    [N,~] = size(results(1).dis2t);
    tx = 0:evalParams(4):(N-1)*evalParams(4);
    for i=1:num
        ty(:,i)=results(i).dis2t';
    end
    y = results(1).dis2t';
    plot(tx,ty);
    axis()
    xlabel('time(s)');
    ylabel('Distance to target(m)')
    
    figure('Name','Nearest Neighbor Distance');
    [N,~] = size(results(1).dis2d);
    dx = 0:evalParams(4):(N-1)*evalParams(4);
    for i=1:num
        dy(:,i)=results(i).dis2d';
    end
%     y = results(1).dis2t';
    plot(dx,dy);
    axis([0 60 0 20]);
    xlabel('time(s)');
    ylabel('Distance to nearest neighbor(m)')
end


