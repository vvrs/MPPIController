% close all
clear all
clc

%tracker
syms x0 x0_ x1 x2 wr vr

%positive control paramter 
si_v = 1.5;
si_w = 0.2;
m = 1;

%system constraint 
vmin = 8;
vmax = 13;
wmax = 0.671;

% system constant
k1 = 2;
gamma0 =0.5; 
gamma1 =0.5;
gamma2 = 0.5;

%This function will generate eta value for any given condition
eta = get_eta(k1, m, vmin, vmax, wmax, si_v, si_w, gamma0, gamma1, gamma2);

% Time and Time Step
dt=0.01;
Tf = 30;
tsteps= 0:dt:Tf;
N=size(tsteps,2);

% Waypoint of front vehicle
t = [0 6 12 18 24 30];
% points = [-270 85; -215 60; -175 40; -125 27.88; -100 2.42; -50 -20];
points = [-270 85; -215 60; -175 40; -138 35; -110 15; -90 -10];


% Trajectory of Front Vehicle
Cubic_Spline_Curve;
Traj_des_f = Traj_des;
wref_f = wref;
vref_f = vref;
thetaref_f = thetaref;

% Waypoint and Time
t = [0 6 12 18 24 30];
points = [-250 -50; -215 -5; -175 40; -115 20; -70 -20; -10 -10];


% Trajectory Generation Using Cubic Spline
Cubic_Spline_Curve;

% change of variable
pi1 = sqrt(x1^2 + x2^2 + 1);
x0_v = x0_/m - x1/(m*pi1);
x0__v = m*x0 + x1/pi1;


% % initialization of initial input value 
X = zeros(3,N);
U_plot = zeros(2,N);
zstor = zeros(1,N);

X(:,1) = [-315;-10;2.75];
U_plot(:,1) = [0;0];

for i =1:N-1
    t_var = tsteps(i);
  
    %extracting value from trajectory curve
    xdes = Traj_des(1,i);
    ydes = Traj_des(2,i);
    theta = thetaref(i);
    
    %calcualting of theta or si desired and updatation of desired vector
    xdes_vec = [xdes; ydes; theta];
    
    % calculating v reference and w reference
    vf = vref(i);
    wf = wref(i);
    
    % updating xe(error) vector
    x_vec = X(:,i);
    x_dummy = [cos(x_vec(3)) sin(x_vec(3)) 0;...
        -sin(x_vec(3)) cos(x_vec(3)) 0;...
        0 0 1];
    xe_vec = x_dummy*(xdes_vec - x_vec);
    
    % updating the error vector as x0_
    x0_bar = double(subs(x0__v,[x0, x1, x2], [xe_vec(3), xe_vec(2), -1*xe_vec(1)]));
    Xtemp = [x0_bar; xe_vec(2); -1*xe_vec(1)];
    
    %Distance Between Front and Ego Vehicle
    z = ((X(1,i) - Traj_des_f(1,i))^2 + (X(2,i) - Traj_des_f(2,i))^2)^0.5;
    zstor(i) = z;
%     U = controller(Xtemp(1), Xtemp(2), Xtemp(3), vf, wf, eta);
%     U = Controller_CBF(Xtemp(1), Xtemp(2), Xtemp(3), vf, wf, m, z, vref_f(i), U_plot(2,i));
    U = Controller_CBF_withW(Xtemp(1), Xtemp(2), Xtemp(3), vf, wf, m, z, vref_f(i), U_plot(2,i));
    
    
    wc = wf - U(1);
    vc = U(2) + vf*cos(xe_vec(3));
%     if abs(wc)>pi
%         wc = rem(wc,pi);
%     end
%     if tsteps(i) > 10 && tsteps(i) < 20
%         wc = U_plot(1,i);
%         vc = U_plot(2,i);
%     end
    U_plot(:,i+1) = [wc ; vc];
    
    X(3,i+1) = X(3,i)+ dt*wc;
    X(1,i+1) = X(1,i)+ dt*vc*cos(X(3,i));
    X(2,i+1) = X(2,i)+ dt*vc*sin(X(3,i));
    
end

figure;
plot(x,y,'o',xq,yq,'-');
axis([-350.0 0.0 -100.0 100.0]);
title('y vs. x');
hold on

p = plot(X(1,1),X(2,1),'o','MarkerFaceColor','red');
% p2 = plot(Traj_des_f(1,1),Traj_des_f(2,1),'o','MarkerFaceColor','blue');
for k = 2:length(X)
    p.XData = X(1,k);
    p.YData = X(2,k);
%     p2.XData = Traj_des_f(1,k);
%     p2.YData = Traj_des_f(2,k);
    drawnow
end
plot(X(1,:), X(2,:),':');
title('Reference and Actual Trajectory of Vehicle');


figure
plot(tsteps, U_plot(1,:), 'LineWidth', 4);
title('Steer Command Vs Time');

figure
plot(tsteps, U_plot(2,:), 'LineWidth', 4);
title('Velocity Command Vs Time');

% z = ((X(1,:) - Traj_des_f(1,:)).^2 + (X(2,:) - Traj_des_f(2,:)).^2).^0.5;
% figure
% plot(tsteps, zstor, 'LineWidth', 4);
% title('Distance Vs Time');

