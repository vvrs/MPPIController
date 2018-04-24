function [tmeasure, states_history, u_opt] = MPPIControl(system_dynamics, param, running_cost, terminal_cost)


% Optmization hyperparameters
timesteps = param.N; %Horizon(in the paper its called timesteps)
samples = param.K;  %Number of samples
iterations = param.iterations; % Number of iterations
xmeasure = param.x_t0;    %Initial conditions
x_desired = param.x_desired; 
tmeasure = param.t0;    % Initial time
dT = param.dT;    % Timestep
u = param.u; % Initial input sequence
u_UB = param.u_UB; %Control upper bound
u_LB = param.u_LB; %Control lower bound

% Control hyperparameters
lambda = param.lambda; %Inverse temperature
sigma = param.sigma; %Covariance
gamma = param.gamma;
alpha = param.alpha;

system_noise = sigma*ones([1,size(xmeasure,2)]);

% Initialize matrices
states_history = [];
cost = 0;
u_opt = zeros(1,size(u,2));
error_history = [];
dW_history = [];
states_history = [states_history; xmeasure];
v = zeros(timesteps-1, size(u,2));
rng('default');
rng(1);

% Begin MPC iterations
for loop = 0:1:iterations


    minS = 99999999999999999;
    % Initialize the costs to zero
    S = zeros(1, samples-1);
    % Get a normal distributed noise for the input control
    noise = normrnd(0,sigma,[size(u,2),timesteps,samples-1]);


    close(figure(1))

    % Initialize forward sampling
    for k = 1:1:samples-1
        save_trajectory = zeros(timesteps,size(xmeasure,2));
      
        % Set the initial state for every trajectory
        x_0 = xmeasure(end,:);
        t_0 = tmeasure(end,:);
        
        % Apply the input for the given horizon
        for t = 2:1:timesteps
            % Save trajectory
            
            save_trajectory(t,:) = x_0;
            
            if k < (1 - alpha)*samples   
                v(t-1,:) = u(t-1,:) + noise(:,t-1,k)';            
            else
                v(t-1,:) = noise(:,t-1,k)';     
            end

            for i=1:1:size(u,2)
                if  v(t-1,i) > u_UB(i)
                    v(t-1,i) = u_UB(i); 
                elseif v(t-1,i) < u_LB(i)
                    v(t-1,i) = u_LB(i); 
                end
            end
            
            %  Set it to the system
            l = t-1;
                      
            dW = normrnd(0,system_noise,[1,size(xmeasure,2)]);          
            
            [t_, x_] = ode45(@(t,x)DynamicModel(t, x ,v(l,:),dW), [t_0 t_0+dT], x_0);
            x_0 = x_(end,:);
            t_0 = t_(end,:);
            
            % Calculate the cost
            [cost error] = running_cost(x_0,x_desired);
            S(k) = S(k) + cost + gamma*u(t-1,:)*inv(diag(ones(1, size(u,2)))*sigma)*v(t-1,:)';
%             S(k) = 0;
            %             Set an upper bound for the cost
            %             if S(k) > 10000
            %                 S(k) = 10000;
            %             end
           
            
        end
%       Terminal cost
        save_trajectory = [save_trajectory; x_0];
        S(k) = S(k) + terminal_cost(x_0,x_desired);
        
        
        % Plot all trajectories
        if S(k) < minS
            minS = S(k);
            % Display relevant information
            disp(['Sample: ', num2str(k), ' iteration: ', num2str(loop)...
            , ' x: ' num2str(xmeasure(1)), ' m',...
            ' x_dot: ' num2str(xmeasure(2)),...
             ' phi: ' num2str(xmeasure(3)), ' degrees', ...  % ' phi: ' num2str(rad2deg(xmeasure(3))), ' degrees', ...           
           ' phi_dot: ' num2str(xmeasure(4)),...
            ' Last Input: ', num2str(u_opt(end,:)), ' Sample Cost: ' num2str(S(k))])
        end
    end

%     Calculate the weights
    weights = calculate_weights(S,lambda,samples);

    
    %     Push the control distibution to the optimal one
    weights_vector = zeros(timesteps,1);
    for t=1:1:timesteps
  
        weights_sum = zeros(1,size(u,2));
  
        for k=1:1:samples-1
            weights_sum = weights_sum + (weights(k)*noise(:,t,k))';
        end
        
        weights_vector(t,:) = weights_sum;
    end
                    
    for t=1:1:timesteps
        u(t,:) = u(t,:) + weights_vector(t,:);

        for i = 1:1:size(u,2)
            if(u(t,i) < u_LB(i))
                u(t,i) = u_LB(i);
            elseif (u(t,i) > u_UB(i))
                u(t,i) = u_UB(i);
            end
        end
    end
        
    if mod(loop,1)==0        
        % save the first optimal input
        u_opt = [u_opt; u(1,:)];
    
   
        %   Set the optimal input to the system  
        %     u(1,:) = u(1,:) +  normrnd(0,sigma,[1,1]);

        for state=1:1:size(xmeasure,2) 
                dW(state) = sqrt(dT)*normrnd(0,system_noise(state),[1,1]);
        end

        [t_f, next_state] = ode45(@(t,x)DynamicModel(t, x ,u(1,:),dW), [tmeasure tmeasure+dT], xmeasure);
        x_open = xmeasure;
        t_open = tmeasure;
        
        %     Add noise to the output
        for state=1:1:size(next_state,2) 
            xmeasure(:,state) = next_state(end,state);%  +  normrnd(0,system_noise(state),[1,1]);
        end

        
        tmeasure = t_f(end,:);
     
        [cost, error_running] = running_cost(x_0,x_desired);
        [cost, error_terminal] = terminal_cost(x_0,x_desired);
        error = error_running + error_terminal;
        error_history = [error_history;error];
        dW_history = [dW_history;
                        dW];
        % Save the state
        states_history = [states_history; xmeasure];            

        for t = 1:1:timesteps-1
            u(t,:) = u(t+1,:);
        end
        
        %     Initialize the new last input with a zero
        u(timesteps,:) = 0;
    end 
end

function weights = calculate_weights(S,lambda,samples)

    %     Calculate weights
    n = 0;
    weights = zeros(1,samples);
    beta = min(S);
    for k=1:1:samples-1
        n = n + exp(-(1/lambda)*(S(k) - beta));
    end
    for k=1:1:samples-1
        weights(k) = (exp(-(1/lambda)*(S(k)-beta)))/n;
    end
