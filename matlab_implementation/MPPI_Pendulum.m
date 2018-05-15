clear all;
clc;



K = 1000;
N = 100;
iteration = 500;
param.dt = 0.02;

% System Paramters Given in paper
param.mc = 1;
param.mp = 0.01;
param.l = 1;
param.g = 9.81;

% Variance and Lamda
param.lambda = 10;
param.variance = 10;
param.R = 1;

% param.lambda = 1/(param.variance*K);

% Initial State
x_init = [0 0 0 0];

% Final state for Cart Pole
x_fin = [0 0 pi 0];

% Variables To store the system state
X_sys = zeros(4,iteration+1);
U_sys = zeros(1,iteration);
cost  = zeros(1,iteration);
cost_avg = zeros(1,iteration);

X_sys(:,1) = x_init;

% Initialization of Variables
x = zeros(4,N);
delta_u = zeros(N,K);
u_init = 1;

X_sys(:,1) = x_init;

% Initialization of input for N time horizone
u = zeros(1,N);

% MPPI Loop
for j = 1: iteration
    % Initialization of Cost for K Samples
    Stk = zeros(1,K);
    
    % Calculating cost for K samples and N finite horizone
    for k = 1:K
        x(:,1) = x_init;
        for i = 1:N-1
            delta_u(i,k) = param.variance*(randn(1));
            x(:,i+1) = x(:,i) + Pendulam_Dynamics(x(1,i), x(2,i), x(3,i),...
                x(4,i), (u(i)+delta_u(i,k)), param)*param.dt;
                Stk(k) = Stk(k) + cost_function(x(1,i+1), x(2,i+1), x(3,i+1), ...
                    x(4,i+1),(u(i)+ delta_u(i,k)),param);
                
           % implementing updated cost function given in paper Aggressive
           % Driving with MPPI control
%             Stk(k) = Stk(k) + cost_function_updat(x(1,i+1), x(2,i+1), x(3,i+1), ...
%                 x(4,i+1),u(i),delta_u(i,k),param);
        end
        delta_u(N,k) = param.variance*(randn(1));
        
    end
    
    % Average cost over iteration
    cost_avg(j) = sum(Stk)/K;
    
    % Updating lambda as function of R which Lambad = 1/R here R = Cost
    % function
%     param.lambda = 1000*cost(j) + 1;

    % Updating the control input according to the expectency over K sample
    % trajectories
    for i = 1:N
        u(i) = u(i) + totalEntropy(Stk(:) , delta_u(i,:),param);
    end
    
    % Input to the system 
    U_sys(j) = u(1);
    
    % System Updatation because of input
    X_sys(:,j+1) = X_sys(:,j) + Pendulam_Dynamics(X_sys(1, j), X_sys(2,j),...
        X_sys(3,j), X_sys(4,j), u(1), param)*param.dt;
    
    % Calculating state cost function
    cost(j+1) = cost_function(X_sys(1,j+1), X_sys(2,j+1), X_sys(3,j+1),...
        X_sys(4,j+1),(u(i)+delta_u(i,k)),param);
    
    % Input updatation for next time step
    for i = 1:N-1
        u(i) = u(i+1);
    end
    
    % Updating the input in Last time step
    u(N) = u_init;
    
    % initial state for calculating the expectency over trajectory in next
    % step
    x_init = X_sys(:,j+1);  
end

%% plot the cart pole state X and theta
figure
plot(X_sys(1,:))
hold on
plot(X_sys(3,:))
title('X and Theta');
ylabel('X and Theta');
xlabel('Iteration');
legend('X--', 'Theta');


% Animation of Cart Pole
figure;
animatePendulumCart;