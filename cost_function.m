function [S] = cost_function(p, p_dot, theta, theta_dot)

    S = p^2 + 500*(1+cos(theta))^2 + theta_dot^2 + p_dot^2;
%     if isnan(S)
%                p, p_dot, theta, theta_dot
%     end
    S = S;
end
