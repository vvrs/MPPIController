import numpy as np

"""
mc = param.mc;
mp = param.mp;
l = param.l;
g = param.g;
dX(1) = x_dot;
dX(2) = (u + mp*sin(theta)*(l*theta_dot^2 + g*cos(theta)))/(mc+mp*sin(theta)^2);
dX(3) = theta_dot;
% dX(3) = rem(theta_dot,2*pi);
dX(4) = (-u*cos(theta) - mp*l*theta_dot^2*cos(theta)*sin(theta)...
    -(mc+mp)*g*sin(theta))/(mc+mp*sin(theta)^2);
% if(isnan(dX(1)))
%     u
%     dX
%     return;
% end

dX = dX';
end
"""

l = 0.25
g = 9.81


def pendulum_dynamics(x, x_dot, theta, theta_dot, u, mc, mp):
    dx = [x_dot,
          (u + mp * np.sin(theta) * (l * theta_dot ** 2 + g * np.cos(theta))) / (mc + mp * np.sin(theta) ** 2),
          theta_dot,
          (-u * np.cos(theta) - mp * l * theta_dot ** 2 * np.cos(theta) * np.sin(theta) - (mc + mp) * g * np.sin(
              theta)) / (mc + mp * np.sin(theta) ** 2)]
    return np.array(dx)


"""
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
"""


def cost_function(p, p_dot, theta, theta_dot, u):
    dt = 0.02
    S = (20 * np.abs(p) ** 2 + 100 * (1 + np.cos(theta)) ** 2 + 1 * theta_dot ** 2 + 1 * p_dot ** 2) * dt / 10
    if p > 8:
        S += 0
    return S


"""
function [entropy] = totalEntropy(Sk , del_uk)
%     Sk = Sk./sum(Sk);
%     alpha = min(Sk);
%     Sk = Sk-alpha;
%     del_uk = del_uk./sum(del_uk);
    n = length(Sk);
    lambda = 10;
    sum1 = 0;
    sum2 = 0;
    for i = 1:n
        sum1 = sum1 + exp(-(1/lambda)*Sk(i))*del_uk(i);
        sum2 = sum2 + exp(-(1/lambda)*Sk(i));
    end
    entropy = sum1/sum2;

end
"""


def total_entropy(Sk, del_uk):
    n = len(Sk)
    lambda_ = 10
    sum1 = 0
    sum2 = 0
    for i in range(n):
        sum1 += np.exp(-(1 / lambda_) * Sk[i]) * del_uk
        sum2 += np.exp(-(1 / lambda_) * Sk[i])

    entropy = sum1 / sum2
    return entropy


"""

"""

def main_loop():
    K = 1000
    N = 50
    iteration = 200
    dt = 0.02

    mc = 1
    mp = 0.01
    l = 0.25
    g = 9.81
    lambda_ = 1
    variance = 100
    x_init = np.array([0,0,0,0])
    x_fin = np.array([0,0,np.pi,0])

    X_sys = np.zeros((4, iteration+1))

    U_sys = np.zeros((1, iteration))
    cost = np.zeros((1, iteration))

    X_sys[:, 1] = x_init

    u = np.zeros((N,1))
    x = np.zeros((4, N))
    delta_u = np.zeros((N, K))
    u_init = 1

    # MPPI LOOP
    x = np.zeros((4,N))

    for j in range(iteration):
        Stk = np.zeros((1, K))

        for k in range(K):
            x[:, 1] = x_init
            for i in range(N-1):
                print("i>>",i)
                delta_u[i, k] = variance * float(np.random.normal(1,1,1))
                x[:, i+1] = x[:, i] + pendulum_dynamics(x[0, i], x[1, i], x[2, i], x[3, i], (u[i] + delta_u[i, k]), mc, mp)*dt
                Stk[k] += cost_function(x[0, i+1], x[1, i+1], x[2, i+1], x[3, i+1], (u[i] + delta_u[i, k]))
            delta_u[N-1, k] = variance * float(np.random.normal(1,1,1))

        for i in range(N):
            u[i] += total_entropy(Stk[:], delta_u[i,:])
        U_sys[j] = u[0]
        X_sys[:, j+1] = X_sys[:, j] + pendulum_dynamics(X_sys[0, j], X_sys[1, j], X_sys[2, j], X_sys[3, j], u[1],mc,mp)*dt
        cost[j + 1] = cost_function(X_sys[0, j+1], X_sys[1, j+1], X_sys[2, j+1], X_sys[3, j+1], (u[i]+delta_u[i, k]))
        for i in range(N):
            u[i] = u[i+1]
        u[N-1] = u_init
        x_init = X_sys[:, j+1]




main_loop()

print("Hello world")
