function VelocityVectorApproach_multi_4
    close all;
    clear all;
    clc;
    disp('Velocity Vector Program - multi drones test start!!');

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% init %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    num=4;%drone num
    scale = 12;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% random %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    quad_init_x=scale*rand(num,1);%init pos
    quad_init_y=scale*rand(num,1);
    goal = 0.8*scale*rand(num,2);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% case 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     quad_init_x=[2;8;2;8];
%     quad_init_y=[2;8;8;2];
%     goal = [8 8;
%             2 2;
%             8 2;
%             2 8;
%             ];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% case 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     quad_init_x=[2;8;2;8];%init pos
%     quad_init_y=[2;2;8;8];

%     load("demo1.mat");

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
    
    obstacleR=0.1;% parameter r[m]
                  % dimension of the drones

    % velocity params
    Vm = 1.0; %Vmax [m/s]
    Vacc = 1; %acc [m/ss]
    Wm = toRadian(20.0);
    Wacc = toRadian(40.0);
    Rv = 0.01; % resolution
    Rw = 0.01;
    Kinematic = [Vm,Vacc,Wm,Wacc,Rv,Rw];

    % zone params
    dangerR = 1.0; % paranmeter R
    omega = 2.0;   % parameter omega
    zoneParam = [dangerR,omega*dangerR];

    global dt; dt=0.05;% time[s]
    evalParams=[0.5,0.2,0.1,0.05];

    area=[-1 scale -1 scale];% simulation area [xmin xmax ymin ymax]

    % init status[xi,yi,yaw,v,w,stop]
    x=[quad_init_x(1,1) quad_init_y(1,1) atan2((goal(1,2)-quad_init_y(1,1)),(goal(1,1)-quad_init_x(1,1))) 0 0 0;
       quad_init_x(2,1) quad_init_y(2,1) atan2((goal(2,2)-quad_init_y(2,1)),(goal(2,1)-quad_init_x(2,1))) 0 0 0;
       quad_init_x(3,1) quad_init_y(3,1) atan2((goal(3,2)-quad_init_y(3,1)),(goal(3,1)-quad_init_x(3,1))) 0 0 0;
       quad_init_x(4,1) quad_init_y(4,1) atan2((goal(4,2)-quad_init_y(4,1)),(goal(4,1)-quad_init_x(4,1))) 0 0 0;]';
   
    drones=x(1:2,1:num)';
    
    for di = 1:num
        results.obsi = [];
        results.quad = [];
        results.traj = [];
    end
    
    % arrived drone -> 1
    arrived = zeros(num,1);

    tic;
    
%     writerObj=VideoWriter('simulation.avi');   
%     open(writerObj); 
    
    for i=1:5000
        
        for di = 1:num
            if norm(x(1:2,di)-goal(di,:)') > obstacleR 
                obsi = drones;
                obsi(di,:) = [];
                results(di).obsi = obsi;
                [x(:,di),results(di).traj] = multiDroneApproach(x(:,di),Kinematic,goal(di,:),zoneParam,obs,results(di).obsi,obstacleR,evalParams);
                drones=x(1:2,1:num)';
            else
                obs = [obs;x(1,di) x(2,di)];
                arrived(di,1) = 1;
            end
            results(di).quad=[results(di).quad; x(:,di)'];
        end

        if all(arrived) == 1
            disp('Arrive Goal!!');break;
        end

        %====Animation====
        hold off;
        ArrowLength=0.2;
        
        for di = 1:num
            quiver(x(1,di),x(2,di),ArrowLength*cos(x(3,di)),ArrowLength*sin(x(3,di)),'ok');hold on;
%             plotCircle(x(1,di),x(2,di),zoneParam);hold on;
            plot(results(di).quad(:,1),results(di).quad(:,2),'-b');hold on;
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
    toc;
end




