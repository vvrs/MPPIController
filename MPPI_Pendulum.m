% clear all;
clc;

K = 1000;
N = 50;
iteration = 200;
param.dt = 0.02;

param.mc = 1;
param.mp = 0.01;
param.l = 0.25;
param.g = 9.81;

param.lambda = 1;
param.variance = 100;

% Initial State
x_init = [0 0 0 0];
x_fin = [0 0 pi 0];

% to store the system state
X_sys = zeros(4,iteration+1);
U_sys = zeros(1,iteration);
cost  = zeros(1,iteration);

X_sys(:,1) = x_init;

% Initial Input
load('u_init');
u = Force;

x = zeros(4,N);
Stk = ones(N,K);
delta_u = zeros(N,K);
u_init = 1;

X_sys(:,1) = x_init;


% MPPI Loop
for j = 1: iteration
    Stk = zeros(1,K);
    
    for k = 1:K
        x(:,1) = x_init;
        for i = 1:N-1
            delta_u(i,k) = param.variance*(randn(1));
            x(:,i+1) = x(:,i) + Pendulam_Dynamics(x(1,i), x(2,i), x(3,i), x(4,i), (u(i)+delta_u(i,k)), param)*param.dt;
            Stk(k) = Stk(k) + cost_function(x(1,i+1), x(2,i+1), x(3,i+1), x(4,i+1),(u(i)+delta_u(i,k)));
        end
        delta_u(N,k) = param.variance*(randn(1));
        
    end
    for i = 1:N
        u(i) = u(i) + totalEntropy(Stk(:) , delta_u(i,:));
    end
    
    U_sys(j) = u(1);
    X_sys(:,j+1) = X_sys(:,j) + Pendulam_Dynamics(X_sys(1, j), X_sys(2,j), X_sys(3,j), X_sys(4,j), u(1), param)*param.dt;
    cost(j+1) = cost_function(X_sys(1,j+1), X_sys(2,j+1), X_sys(3,j+1), X_sys(4,j+1),(u(i)+delta_u(i,k)));
    for i = 1:N-1
        u(i) = u(i+1);
    end
    u(N) = u_init;
    x_init = X_sys(:,j+1);  
end
