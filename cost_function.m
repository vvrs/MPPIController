function [S] = cost_function(p, p_dot, theta, theta_dot, u)
    
    dt = 0.02;
    S = (20*abs(p)^2 + 100*(1+cos(theta))^2 + 1*theta_dot^2 + 1*p_dot^2)*dt/10;
     if p>8
         S = S + 0;
     end

end
