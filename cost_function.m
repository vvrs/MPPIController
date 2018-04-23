function [S] = cost_function(p, p_dot, theta, theta_dot, u)
    
    dt = 0.02;
%     S = (0.1*p^2 + 0.5*(1+cos(theta))^2 + 0.1*theta_dot^2 + p_dot^2+ 100*abs(p+p_dot) +...
%         100*abs(abs(pi - theta) + theta_dot))*dt/10000;
%     S = (10*abs( p+ 0.1*p_dot) + 100*abs(abs(pi - theta) + 0.1*theta_dot))*dt/10000;
%      S = (40*p^2 + 50*(1+cos(theta))^2 + 0.1*theta_dot^2 +0.01*(p+p_dot)^2 + ...
%          0.01*(p_dot - theta_dot)^2)*dt/1000;
%      S = (10*p^2 + 50*(1+cos(theta))^2 + 0.1*theta_dot^2 +0.01*(p+p_dot)^2)*dt/1000;
%      S = (p^2 + 5*(1+cos(theta))^2 + 0.0001*theta_dot^2 +0.0001*p_dot^2 + 0.1*u^2*1e-6)*dt;
%      S = ((p)^2 + 10*(pi - abs(theta))^2 + 1*abs(theta_dot)^1/2 + 1*abs(p_dot)^0.5)*dt/100;
%      S = (10*abs(p)^2 + 100*(1+cos(theta))^2 + 0.1*theta_dot^2 + 0.1*p_dot^2)*dt/10;
%     result S = (10*abs(p)^2 + 100*(1+cos(theta))^2 + 1*theta_dot^2 + 1*p_dot^2)*dt/10;
    S = (20*abs(p)^2 + 100*(1+cos(theta))^2 + 1*theta_dot^2 + 1*p_dot^2)*dt/10;
%     S = (1*abs(p)^2 + 500*(1+cos(theta))^2 + 1*theta_dot^2 + 1*p_dot^2)*dt/10;
     if p>8
         S = S + 0;
     end

%     if isnan(S
%                p, p_dot, theta, theta_dot
%     end
end
