function velocityParam
    r=5; % drone r
    R=12; % danger zone R
    w=3; % sensor zone wR
    step = 1;
    space = 55;
    % x=0:0.01:10;
    % y=x.*(x>=0&x<4)+2*(x>=4&x<6)+(5-x/2).*(x>=6&x<8)+1*(x>=8);
    di=r:step:space;
    alpha=0*(di>r&di<=R)+(1-cos(pi*(di-R)/(w*R-R)))/2.*(di>R&di<=w*R)+1*(di>w*R);
    beta = (1+cos(pi*(di-r)/(R-r)))/2.*(di>r&di<=R)+0*(di>R&di<=w*R)+0*(di>w*R);
    gamma = (1-cos(pi*(di-r)/(R-r)))/2.*(di>r&di<=R)+(1+cos(pi*(di-R)/(w*R-R)))/2.*(di>R&di<=w*R)+0*(di>w*R);
    plot(di,alpha,'r','linewidth',1.5);
    hold on;  
    plot(di,beta,'b','linewidth',1.5);
    hold on;
    plot(di,gamma,'g','linewidth',1.5);
    grid on
end