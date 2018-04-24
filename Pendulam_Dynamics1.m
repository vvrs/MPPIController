function Xd = Pendulam_Dynamics1(x, x_dot, theta, theta_dot, u, param)

%Define Parameters
m = param.mp;    %Pendulum point mass
M = param.mc;    %Cart mass
L = param.l;    %Pendulum length (to point mass)
g = param.g;    %Gravity

%Define input states:
% % x = X(1);    %Horizontal position of the cart
v = x_dot;      %Horizontal velocity of the cart
th = theta;     %Angle of the pendulum: 0 = straight up, pi = straight down
w = theta_dot;      %Angular rate of the pendulum
F = u;

%Evaluate sines and cosines
Sin = -sin(th);
Cos = -cos(th);

% Analytic Inverse of Coeff via Mathematica:
Det = m*L*(M*Cos^2 + (m+M)*Sin^2);
inv_Det = 1/Det; 

%Broken Out UnKnowns (By Hand)
vd = (F*m*L +  m*L*w^2*Sin*m*L*Sin^2 + (m*g + m*L*w^2*Cos)*m*L*Cos*Sin)*inv_Det;%Translational acceleration
wd = (-F*m*Cos + m*L*w^2*Sin*M*Cos  +  -(m*g + m*L*w^2*Cos)*(M+m)*Sin)*inv_Det;%Angular Acceleration

%Defined derivatives:
xd = v;
thd = w;

%Express as a vector
Xd = [xd;vd;thd;wd];

end
