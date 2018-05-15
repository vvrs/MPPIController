function dX = Pendulam_Dynamics(x, x_dot, theta, theta_dot, u, param)

mc = param.mc;
mp = param.mp;
l = param.l;
g = param.g;

% States of the Pendulum dX(1) is X_dot dX(2) is X double dor, dX(3) is
% Theta_dot and dX(4) is theta double dot
dX(1) = x_dot;

dX(2) = (u + mp*sin(theta)*(l*theta_dot^2 + g*cos(theta)))/(mc+mp*sin(theta)^2);

dX(3) = theta_dot;

dX(4) = (-u*cos(theta) - mp*l*theta_dot^2*cos(theta)*sin(theta)...
    -(mc+mp)*g*sin(theta))/(mc+mp*sin(theta)^2);


dX = dX';
end

