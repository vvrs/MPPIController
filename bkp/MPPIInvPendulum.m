clc
clear all;
close all;

% Parameters
param.N = 20; % length of finite horizon
param.K = 100; % # samples 
param.iterations = 100; % replacement to while(task_completed)
param.x_t0 = [0 0 pi 0]; % initial state
param.x_desired = [0 0 0 0]; % final state
param.t0 = 0; % initial time
param.dT = 0.1; % time step

param.u_init_seq = zeros(30,1);
param.u = param.u_init_seq;
param.u_init = 0;

% input bounds
param.u_UB = 10;
param.u_LB = -10;

% parameters to model uncertainty
param.sigma = 0.2;
param.mu = 0;

% control parameters
param.lambda = 1; % temperature parameter (?)
param.alpha = 0.05; % Base distribution (0-1) (?)
param.gamma = param.lambda*(1-param.alpha); % inv(H)*G (?)

[t_all, x_all, u_all] = MPPIControl(@DynamicModel, param, @running_cost, @terminal_cost);

figure;
plot(u_all);

figure;
legend;
plot(x_all(1,:));
hold on
plot(x_all(2,:));
hold on
plot(x_all(3,:));
hold on
plot(x_all(4,:));



function [J, error] = running_cost(x, x_desired,u)

 J_(1) = 10*(x(1))^2 ;
 J_(2) = 500*(1 + cos(x(3)+pi))^2;
%  J_(2) = 500*(x(3))^2;
 J_(3) = 1*(x(2))^2;
 J_(4) = 15*(x(4))^2;
 J_(5) = u*u;
 J = sum(J_);
 
 error(1) = sqrt((x(1) - x_desired(1))^2);
 error(2) = sqrt((x(2) - x_desired(2))^2);
 error(3) = sqrt((x(3) - x_desired(3))^2);
 error(4) = sqrt((x(4) - x_desired(4))^2);
end

function [J, error] = terminal_cost(x, x_desired)


J = 0;
 error(1) = 0*sqrt((x(1) - x_desired(1))^2);
 error(2) = 0*sqrt((x(2) - x_desired(2))^2);
 error(3) = 0*sqrt((x(3) - x_desired(3))^2);
 error(4) = 0*sqrt((x(4) - x_desired(4))^2);
end