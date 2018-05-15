function [S] = cost_function(p, p_dot, theta, theta_dot, u, param)
    dt = param.dt;
    
    % P is position of Cart Pole Theta is angle theta_dot is change in
    % angle and p_dot is change in position
    S = (6*(p)^2 + 12*(1+cos(theta))^2 + 0.1*theta_dot^2 + 0.1*p_dot^2)*dt;
    
    % Cost function given in paper Model Predictive Path Integral Control
    % from theory to parallel computation
%     S = (1*(p)^2 + 500*(1+cos(theta))^2 + 1*theta_dot^2 + 1*p_dot^2)*dt;


end
