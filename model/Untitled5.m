k = 0.5; % slope
b = 1; % interception
x1 = 3;
y1 = 2;

x2 = (k*y1+x1-k*b)/(1+k*k);
y2 = k*x2+b;

figure
hold on
plot(x1,y1,'ro')
plot(x2,y2,'r.','MarkerSize',15)
plot(0:0.01:abs(x2), k*(0:0.01:abs(x2))+b, 'b-')
plot([x1,x2],[y1,y2],'k-')
axis equal
hold off

disp([x2,y2])