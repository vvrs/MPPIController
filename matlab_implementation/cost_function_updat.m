function [S] = cost_function_updat(p, p_dot, theta, theta_dot, u, delta_u, param)

% implementing updated cost function given in paper Aggressive Driving with MPPI control
    dt = param.dt;
    S = (6*(p)^2 + 12*(1+cos(theta))^2 + 0.1*theta_dot^2 + 0.1*p_dot^2)*dt;
%     S = (1*(p)^2 + 500*(1+cos(theta))^2 + 1*theta_dot^2 + 1*p_dot^2)*dt;
%     param.lambda
%     param.R
%     u
%     param.variance
%     delta_u
    S = S + 0.5*param.R*u^2 + param.lambda*u*delta_u + 0.5*param.lambda*(1-(1/param.variance))*delta_u^2;


end
