clear all;
clc;

K = 500;
N = 1000;
iteration = 100;
param.dt = 0.01;

param.mc = 1;
param.mp = 0.01;
param.l = 0.25;
param.g = 9.81;
% param.alpha = 0.05;
% param.gamma = param.lambda*(1-param.alpha);
param.lambda = 1;
param.variance = 2000;

% Initial State
x_init = [0 0 0 0];
x_fin = [0 0 pi 0];

% to store the system state
X_sys = zeros(4,iteration+1);

% Initial Input
u = zeros(1, N);
x = zeros(4,N);
Stk = ones(N,K);
delta_u = zeros(N,K);
u_init = 0;

X_sys(:,1) = x_init;

% MPPI Loop
for j = 1: iteration
    x(:,1) = x_init;
    for k = 1:K
        for i = 1:N-1
            delta_u(i,k) = param.variance*(randn(1));
            x(:,i+1) = x(:,i) + Pendulam_Dynamics(x(1,i), x(2,i), x(3,i), x(4,i), (u(i)+delta_u(i,k)), param)*param.dt;
            Stk(i+1,k) = Stk(i,k) + cost_function(x(1,i), x(2,i), x(3,i), x(4,i));
        end
        delta_u(N,k) = param.variance*(randn(1));
        
    end
    for i = 1:N
        u(i) = u(i) + totalEntropy(Stk(i,:) , delta_u(i,:));
        u(i);
    end
%     u(1)
%     u(2)
    X_sys(:,j+1) = X_sys(:,j) + Pendulam_Dynamics(X_sys(1, j), X_sys(2,j), X_sys(3,j), X_sys(4,j), u(1), param)*param.dt;

    for i = 1:N-1
        u(i) = u(i+1);
    end
    u(N) = u_init;
    x_init = X_sys(:,j+1);
        
end
