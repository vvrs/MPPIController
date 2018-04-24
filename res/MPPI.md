### Definitions
**Trajectory:** A sequence of states $x_t$, actions $u_t$ and associated uncertainty $\sigma_{t,n}$ at time step $t$. 

Let $s_{t,n} = \left(x_{t,n},u_{t,n},\sigma_{t,n}\right)$. Here, $n$ indexes a trajectory. 

Cost function for a trajectory $n$ starting from timestep $i$ until $T$, i.e.  

$\tau_{i,n} = \left[s_{i,n},s_{i+1,n},s_{i+2,n},.....,s_{T,n}\right]:$ is $$S_{i} = S\left(\tau_{i,n}\right) = \phi_{T}\left(x_T\right) + \sum_{t=i}^{T-1} r_t\left(x_t,u_t,\sigma_t\right)$$