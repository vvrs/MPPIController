function [S] = cost_function(p, p_dot, theta, theta_dot, u, param)
    dt = param.dt;
    S = (4*(p)^2 + 12*(1+cos(theta))^2 + 0.1*theta_dot^2 + 0.1*p_dot^2)*dt;
%     S = (1*(p)^2 + 500*(1+cos(theta))^2 + 1*theta_dot^2 + 1*p_dot^2)*dt;
end
