
% t = [0 6 12 18 24 30];
% points = [-250 -50; -215 -5; -175 40; -115 20; -70 -20; -10 -10];
x = points(:,1);
y = points(:,2);

tq = 0:dt:Tf;
slope0 = 0;
slopeF = 0;
xpol = spline(t,[slope0; x; slopeF]);
xq =ppval(xpol,tq);
xdpol = fnder(xpol,1);
xdq = ppval(xdpol,tq);
xddpol = fnder(xpol,2);
xddq = ppval(xddpol,tq);

ypol = spline(t,[slope0; y; slopeF]);
yq = ppval(ypol,tq);
ydpol = fnder(ypol,1);
ydq = ppval(ydpol,tq);
yddpol = fnder(ypol,2);
yddq = ppval(yddpol,tq);

Traj_des = [xq; yq; xdq; ydq; xddq; yddq];

K = (xdq.*yddq - xddq.*ydq)./(xdq.^2 + ydq.^2).^1.5;
wref = K.*xdq;
wref(1) = 0;

thetaref = atan2(ydq, xdq);

vref = xdq.*cos(thetaref) + ydq.*sin(thetaref);

% plot spline in t-x, t-y and x-y space
% figure;
% plot(t,x,'o',tq,xq,':.');
% axis([-0.5 10.5 -0.5 80]);
% title('x vs. t');
% 
% figure;
% plot(tq,xdq,':.');
% axis([-0.5 10.5 -0.5 16]);
% title('dx vs. t');
% 
% figure;
% plot(tq,xddq,':.');
% axis([-0.5 10.5 -8.5 12]);
% title('ddx vs. t');
% 
% figure;
% plot(t,y,'o',tq,yq,':.');
% axis([-0.5 10.5 -0.5 3.5]);
% title('y vs. t');
% 
% figure;
% plot(tq,ydq,':.');
% axis([-0.5 10.5 -3.0 3.0]);
% title('dy vs. t');
% 
% figure;
% plot(tq,yddq,':.');
% axis([-0.5 10.5 -3.5 3.5]);
% title('ddy vs. t');
% 

% figure;
% plot(tq,wref,':');
% axis([-0.5 10.5 -1 1]);
% title('wr vs. t');
% 
% figure;
% plot(tq,vref,':.');
% axis([-0.5 10.5 -0.5 15]);
% title('vref vs. t');

% figure;
% plot(x,y,'o',xq,yq,'-');
% axis([-350.0 0.0 -100.0 100.0]);
% title('y vs. x');
% hold on
