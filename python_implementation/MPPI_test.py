import numpy as np



l = 0.25
g = 9.81


def pendulum_dynamics(x, x_dot, theta, theta_dot, u, mc, mp):
    dx = [x_dot,
          (u + mp * np.sin(theta) * (l * theta_dot ** 2 + g * np.cos(theta))) / (mc + mp * np.sin(theta) ** 2),
          theta_dot,
          (-u * np.cos(theta) - mp * l * theta_dot ** 2 * np.cos(theta) * np.sin(theta) - (mc + mp) * g * np.sin(
              theta)) / (mc + mp * np.sin(theta) ** 2)]
    return np.array(dx)




def cost_function(p, p_dot, theta, theta_dot, u):
    dt = 0.02
    S = (20 * np.abs(p) ** 2 + 100 * (1 + np.cos(theta)) ** 2 + 1 * theta_dot ** 2 + 1 * p_dot ** 2) * dt / 10
    if p > 8:
        S += 0
    return S




def total_entropy(Sk, del_uk):
    n = len(Sk)
    lambda_ = 10
    sum1 = 0
    sum2 = 0
    for i in range(n):
        sum1 += np.exp(-(1 / lambda_) * Sk[i]) * del_uk[i]
        sum2 += np.exp(-(1 / lambda_) * Sk[i])

    entropy = sum1 / sum2
    return entropy




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
    x_init = np.array([0, 0, 0, 0])
    x_fin = np.array([0, 0, np.pi, 0])

    X_sys = np.zeros((4, iteration+1))

    U_sys = np.zeros(iteration)
    cost = np.zeros(iteration)

    X_sys[:, 1] = x_init

    u = np.zeros(N)
    # x = np.zeros((4, N))
    delta_u = np.zeros((N, K))
    u_init = 1

    # MPPI LOOP
    x = np.zeros((4,N))

    for j in range(iteration):
        Stk = np.zeros(K)

        for k in range(K):
            x[:, 1] = x_init
            for i in range(N-1):
                # print "i>>", i, " k>>", k
                delta_u[i, k] = variance * float(np.random.normal(1, 1, 1))
                x[:, i+1] = x[:, i] + pendulum_dynamics(x[0, i], x[1, i], x[2, i], x[3, i], (u[i] + delta_u[i, k]), mc, mp)*dt
                # cost_val = cost_function(x[0, i+1], x[1, i+1], x[2, i+1], x[3, i+1], (u[i] + delta_u[i, k]))
                # Stk[k] += cost_val
                Stk[k] += cost_function(x[0, i+1], x[1, i+1], x[2, i+1], x[3, i+1], (u[i] + delta_u[i, k]))
            delta_u[N-1, k] = variance * float(np.random.normal(1,1,1))
        # print "-"*80
        for i in range(N):
            u[i] += total_entropy(Stk, delta_u[i, :])
        U_sys[j] = u[0]
        X_sys[:, j+1] = X_sys[:, j] + pendulum_dynamics(X_sys[0, j], X_sys[1, j], X_sys[2, j], X_sys[3, j], u[1],mc,mp)*dt
        cost[j + 1] = cost_function(X_sys[0, j+1], X_sys[1, j+1], X_sys[2, j+1], X_sys[3, j+1], (u[i]+delta_u[i, k]))
        for i in range(N-1):
            u[i] = u[i+1]
        u[N-1] = u_init
        x_init = X_sys[:, j+1]




main_loop()

print("MPPI controller for inverted pendulum cart pole")
