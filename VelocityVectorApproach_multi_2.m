function VelocityVectorApproach_multi_2
    close all;
    clear all;
    clc;
    disp('Velocity Vector Program - multi drones test start!!');

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% init %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    num=2;%drone num
    scale = 12;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% random %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     quad_init_x=scale*rand(num,1);%init pos
%     quad_init_y=scale*rand(num,1);
%     goal = 0.8*scale*rand(num,2);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% case 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     quad_init_x=[0;4;];
%     quad_init_y=[4;0;];
%     goal = [10 6;
%             6 10;
%             ];
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%% case 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     quad_init_x=[0;10;];
%     quad_init_y=[10;0;];
%     goal = [10 0;
%             0 10;
%             ];

    %%%%%%%%%%%%%%%%%%%%%%%%%%% case 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    quad_init_x=[0;0;];
    quad_init_y=[2;10;];
    goal = [10 10;
            10 2;
            ];
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    
%     load("demo1.mat");
    run("parameters.m");

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
           
    if ~isempty(obstacles)
        obs = linkObstacles(obstacles,0.5);
    else
        obs = [];
    end

    area=[-1 scale -1 scale];% simulation area [xmin xmax ymin ymax]

    % init status[xi,yi,yaw,v,w,stop]
    x=[quad_init_x(1,1) quad_init_y(1,1) atan2((goal(1,2)-quad_init_y(1,1)),(goal(1,1)-quad_init_x(1,1))) 0 0 0;
       quad_init_x(2,1) quad_init_y(2,1) atan2((goal(2,2)-quad_init_y(2,1)),(goal(2,1)-quad_init_x(2,1))) 0 0 0;]';
   
    drones=x(1:2,1:num)';
   
    for di = 1:num
        results.obsi = [];
        results.quad = [];
        results.traj = [];
        results(di).dis2t = sqrt((goal(di,2)-x(2,di))^2+(goal(di,1)-x(1,di))^2);
        results(di).dis2d = []; % nearest neib drone
    end
   
%     result.quad1=[]; % save result value
%     result.quad2=[];
    
    % arrived drone -> 1
    flag = zeros(num,1);

    tic;
    
%     writerObj=VideoWriter('simulation.avi');   
%     open(writerObj); 
    
    for i=1:5000

        for di = 1:num
            if norm(x(1:2,di)-goal(di,:)') > 2*droneR 
                obsi = drones;
                obsi(di,:) = [];
                results(di).obsi = obsi;
                [u,traj] = VelocityVectorApproach(x(:,di),Kinematic,goal(di,:),zoneParams,obs,results(di).obsi,droneR,evalParams);
                x(:,di)=updateX(x(:,di),u);
                drones=x(1:2,1:num)';
            else
                obs = [obs;x(1,di) x(2,di)];
                flag(di,1) = 1;
            end
            results(di).dis2t=[results(di).dis2t; sqrt((goal(di,2)-x(2,di))^2+(goal(di,1)-x(1,di))^2)];
            results(di).dis2d=[results(di).dis2d; nearestNeighbour(x(:,di),results(di).obsi)];
            results(di).quad=[results(di).quad; x(:,di)'];
        end
        
        if all(flag) == 1
            disp('Arrive Goal!!');break;
        end

        %====Animation====
        hold off;
        ArrowLength=0.2;
        % drones 
        for di = 1:num
            quiver(x(1,di),x(2,di),ArrowLength*cos(x(3,di)),ArrowLength*sin(x(3,di)),'ok');hold on;
            plot(results(di).quad(:,1),results(di).quad(:,2),'-b');hold on;
            plot(goal(di,1),goal(di,2),'*b');hold on;
        end

        axis(area);
        grid on;
        drawnow;
        
%         frame = getframe;              
%         writeVideo(writerObj,frame); 
        
    end
    toc;
    
    figure('Name','Distance to Target');
    [N,~] = size(results(1).dis2t);
    tx = 0:evalParams(4):(N-1)*evalParams(4);
    for i=1:num
        ty(:,i)=results(i).dis2t';
    end
%     y = results(1).dis2t';
    plot(tx,ty);
    xlabel('t = 0 interval 0.05s');
    
    figure('Name','Nearest Neighbour Distance');
    [N,~] = size(results(1).dis2d);
    dx = 0:evalParams(4):(N-1)*evalParams(4);
    for i=1:num
        dy(:,i)=results(i).dis2d';
    end
%     y = results(1).dis2t';
    plot(dx,dy);
    xlabel('t = 0 interval 0.05s');
    
end




