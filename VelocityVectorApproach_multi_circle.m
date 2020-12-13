function VelocityVectorApproach_multi_circle
    close all;
    clear all;
    clc;
    disp('Velocity Vector Program - multi drones test start!!');

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% init %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    num = 1;%drone num
    scale = 3*num;
    radius = 0.5*scale;
    if scale < 10
        scale = 10;
    end
    if radius < 5
        radius = 5;
    end
    area=[-2*scale 2*scale -2*scale 2*scale];% simulation area [xmin xmax ymin ymax]

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% random %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     centre = radius*ones(1,2);
%     theta = 360/num;
% %     quad_init_x=scale*rand(num,1);%init pos
% %     quad_init_y=scale*rand(num,1);
% %     goal = 0.8*scale*rand(num,2);
%     quad_init_x = zeros(num,1);
%     quad_init_y = zeros(num,1);
%     goal = zeros(num,2);
%     
%     for i = 1:num
%         quad_init_x(i,1) = centre(1,1) + cos(deg2rad(i*theta))*radius;
%         quad_init_y(i,1) = centre(1,2) + sin(deg2rad(i*theta))*radius;
%         goal(i,1:2) = [centre(1,1) + cos(deg2rad(i*theta+180))*radius centre(1,1) + sin(deg2rad(i*theta+180))*radius];
%     end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% circle %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     centre = radius*ones(1,2);
    centre = [0 0];
    theta = 360/num;
    quad_init_x = zeros(num,1);
    quad_init_y = zeros(num,1);
    goal = zeros(num,2);
    
    for i = 1:num
        quad_init_x(i,1) = centre(1,1) + cos(deg2rad(i*theta))*radius;
        quad_init_y(i,1) = centre(1,2) + sin(deg2rad(i*theta))*radius;
%         goal(i,1:2) = [centre(1,1) + cos(deg2rad(i*theta+180))*radius centre(1,1) + sin(deg2rad(i*theta+180))*radius];
        goal(i,1:2) = centre;
    end
   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%     load("demo1.mat");
    run( 'parameters.m' );

    obstacles = [
           centre(1) centre(2)
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
    quad_add_x = -1.0*scale;
    quad_add_y = -1.0*scale;
    x_add = [quad_add_x quad_add_y atan2((goal(1,2)- quad_add_y),(goal(1,1)-quad_add_x)) 0 0 0]';
   
    drones=x(1:2,1:num)';
    
    for di = 1:num
        results(di).obsi = [];
        results(di).quad = []; 
        results(di).traj = [];
        results(di).dis2t = sqrt((goal(di,2)-x(2,di))^2+(goal(di,1)-x(1,di))^2);
        results(di).dis2d = []; % nearest neighour drone
    end
    
    % arrived drone -> 1
    arrived = zeros(num,1);

    tic;
    
%     writerObj=VideoWriter('simulation.avi');   
%     open(writerObj); 
    
    for i=1:2000
        
        if num <= 8 && mod(i,200) == 1
            num = num + 1;
            goal = [goal; [centre(1) centre(2)]];
            x = [x';x_add']';
        end

        for di = 1:num
            if norm(x(1:2,di)-goal(di,:)') > droneR 
                obsi = drones; % other drones as obsi
                obsi(di,:) = []; % remove itself
                results(di).obsi = obsi;
                [ut,obs,ob] = VelocityVectorApproach(x(:,di),Kinematic,goal(di,:),zoneParams,obs,results(di).obsi,droneR,evalParams);
                % update current status
                x(:,di)=updateX(x(:,di),ut);
                drones=x(1:2,1:num)';
            else
                obs = [obs;x(1,di) x(2,di)];
                arrived(di,1) = 1;
            end
            results(di).dis2t=[results(di).dis2t; sqrt((goal(di,2)-x(2,di))^2+(goal(di,1)-x(1,di))^2)];
            results(di).dis2d=[results(di).dis2d; nearestNeighbour(x(:,di),results(di).obsi)];
            results(di).quad=[results(di).quad; x(:,di)'];
        end

        if all(arrived) == 1
            disp('Arrive Goal!!');break;
        end

        %====Animation====
        hold off;
        ArrowLength=droneR;
        
        for di = 1:num
            quiver(x(1,di),x(2,di),ArrowLength*cos(x(3,di)),ArrowLength*sin(x(3,di)),'ok');hold on;
%             plotCircle(x(1,di),x(2,di),zoneParams);hold on;
%             plot(results(di).quad(:,1),results(di).quad(:,2),'-b');hold on;
            plot(goal(di,1),goal(di,2),'*b');hold on;
        end
        
%         plotObstacles(obstacles,dangerR);hold on; 
%         if ~isempty(ob)
%             plot(ob(:,1),ob(:,2),'d');hold on;
%         end

        axis(area);
        grid on;
        drawnow;
        
%         frame = getframe;              
%         writeVideo(writerObj,frame); 
        
    end
%     close(writerObj); 
    toc;

%     figure('Name','Distance to Target');
%     [N,~] = size(results(1).dis2t);
%     tx = 0:evalParams(4):(N-1)*evalParams(4);
%     for i=1:num
%         ty(:,i)=results(i).dis2t';
%     end
% %     y = results(1).dis2t';
%     plot(tx,ty);
%     xlabel('t = 0 interval 0.05s');
%     
%     figure('Name','Nearest Neighbour Distance');
%     [N,~] = size(results(1).dis2d);
%     dx = 0:evalParams(4):(N-1)*evalParams(4);
%     for i=1:num
%         dy(:,i)=results(i).dis2d';
%     end
%     plot(dx,dy);
%     xlabel('t = 0 interval 0.05s');
end

function res = nearestNeighbour(x,obsi)
    [N,~] = size(obsi);
    dis = [];
    for io = 1:N
        dis = [dis;norm(obsi(io,:)-x(1:2)')];
    end
    res = min(dis);
end

