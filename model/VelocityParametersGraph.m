function VelocityParametersGraph
    droneR=0.1; % drone r
    dangerR=0.5; % danger zone R
    w=2; % sensor zone wR
    step = 0.01;
    space = 2.5;
    % x=0:0.01:10;
    % y=x.*(x>=0&x<4)+2*(x>=4&x<6)+(5-x/2).*(x>=6&x<8)+1*(x>=8);
    dis=droneR:step:space;
    
    A = [0  0  1/2 1;  % Target
         1 1/2  0  0;  % Obs
         0 1/2 1/2 0]; % guide
           
    alpha = A(1,1)*(dis<=droneR) + A(1,2)*(dis>droneR&dis<=dangerR)+A(1,3)*(1-cos(pi*(dis-dangerR)/(w*dangerR-dangerR))).*(dis>dangerR&dis<=w*dangerR)+A(1,4)*(dis>w*dangerR);
    beta = A(2,1)*(dis<=droneR) + A(2,2)*(1+cos(pi*(dis-droneR)/(dangerR-droneR))).*(dis>droneR&dis<=dangerR) + A(2,3)*(dis>dangerR&dis<=w*dangerR) + A(2,4)*(dis>w*dangerR);
    gamma = A(3,1)*(dis<=droneR) + A(3,2)*(1-cos(pi*(dis-droneR)/(dangerR-droneR))).*(dis>droneR&dis<=dangerR) + A(3,3)*(1+cos(pi*(dis-dangerR)/(w*dangerR-dangerR))).*(dis>dangerR&dis<=w*dangerR) + A(3,4)*(dis>w*dangerR);
    plot(dis,alpha,'r','linewidth',1.5);
    hold on;  
    plot(dis,beta,'b','linewidth',1.5);
    hold on;
    plot(dis,gamma,'g','linewidth',1.5);
    hold on;
    legend('Vt','Vo','Vg');
    xlabel(' 0 < di < 2.5m')
    grid on
end