%% Online Constraint Tightening in Stochastic Model Predictive Control
% Main script to run the simulation

%% Setup
clear;
clc;
close all;

% Add paths
addpath('system', 'control', 'learning', 'utils');

% Load configuration
config;

rng(params.rng_seed);

%% Initialize system models
fprintf('Initializing system models...\n');

linear_system = LinearSystem(params.A, params.B);


if params.f_nonlinear
    true_system = NonlinearSystem(params.A, params.B, params.nonlinear_factor);
else
    true_system = linear_system;
end

%% Calculate terminal cost matrix P using dlyap
fprintf('Calculating terminal cost matrix P...\n');
params.P = dlyap(params.A', params.Q);
fprintf('Terminal cost matrix P calculated:\n');
disp(params.P);

%% Initialize MPC controller
fprintf('Initializing MPC controller...\n');
mpc = StochasticMPC(linear_system, params);

%% Initialize regression model
fprintf('Initializing Logistic Regression model...\n');
regression_model = LogisticRegression(params);

%% Initialize simulation variables
fprintf('Initializing simulation variables...\n');
% Time
T_sim = params.T_sim;
time = 0:T_sim;

% State and input
nx = params.nx;
nu = params.nu;
state = zeros(nx, T_sim + 1);
input = zeros(nu, T_sim);
state(:, 1) = params.x0;

% Constraint values
constraint_values = zeros(1, T_sim + 1);
constraint_values(1) = params.h_constraint(state(:, 1));

% Iteration counter
iteration = zeros(1, T_sim + 1);

% Gamma history
gamma_history = zeros(1, params.I + 1); 
gamma_history(1) = params.gamma_init(1);  % Assuming all elements of gamma are the same

% Constraint satisfaction probability
constraint_prob = zeros(1, params.I + 1);  
constraint_prob(1) = 0.5;  % Initial guess of 50% probability

% Data collection - maintain a global list
collected_data = struct('gamma', [], 'y', []);

% Debug counters
infeasible_count = 0;
feasible_count = 0;

%% Run simulation
fprintf('Starting simulation with %d iterations...\n', params.I);
fprintf('Each iteration: %d wait steps + %d collection steps = %d total steps\n', ...
    params.T_wait, params.T_col, params.T_wait + params.T_col);
fprintf('Total simulation steps: %d\n', T_sim);

% Initialize iteration counter
i = 0;

% Main simulation loop
for t = 1:T_sim
    
    if mod(t, ceil(T_sim/10)) == 0
        fprintf('Simulation progress: %.1f%% (step %d/%d)\n', 100*t/T_sim, t, T_sim);
    end
    
    
    x_t = state(:, t);
    
   
    [u_t, p_t, info] = mpc.solve(x_t);
    
    if t <= 5
        fprintf('Step %d: exitflag=%d, p_t=%d, u_t=%.4f\n', ...
            t, info.exitflag, p_t, u_t);
    end
    
 
    if info.exitflag > 0
        feasible_count = feasible_count + 1;
    else
        infeasible_count = infeasible_count + 1;
        if infeasible_count < 10  
            fprintf('Warning: Infeasible at step %d, p_t=%d\n', t, p_t);
        end
    end
    
    
    input(:, t) = u_t;
    
  
    w_t = params.noise_uniform_bounds(1) + ...
          (params.noise_uniform_bounds(2) - params.noise_uniform_bounds(1)) * rand(params.nw, 1);
    
    state(:, t+1) = true_system.step(x_t, u_t, w_t);
    
    
    constraint_values(t+1) = params.h_constraint(state(:, t+1));
    
    iteration(t+1) = i;
    
    
    if mod(t, params.T_wait + params.T_col) == 0 && t > 0
     
        i = i + 1;
        
        fprintf('\n----- Iteration %d/%d -----\n', i, params.I);
        fprintf('Feasibility stats: %d feasible, %d infeasible (%.1f%% feasible)\n', ...
            feasible_count, infeasible_count, 100*feasible_count/(feasible_count+infeasible_count));
        
       
        collection_start = t - params.T_col + 1;
        collection_end = t;
        
        y_data = double(constraint_values(collection_start+1:collection_end+1) <= 0);
        
        
        n_satisfied = sum(y_data);
        n_violated = length(y_data) - n_satisfied;
        fprintf('Collected %d data points: %d satisfied, %d violated (%.2f%%)\n', ...
            length(y_data), n_satisfied, n_violated, 100*n_satisfied/length(y_data));
        
       
        current_gamma = mpc.gamma(1);
        fprintf('Current gamma: %.4f\n', current_gamma);
        
        gamma_values = current_gamma * ones(length(y_data), 1);
        

        collected_data.gamma = [collected_data.gamma; gamma_values];
        collected_data.y = [collected_data.y; y_data(:)];
        
            try
                fprintf('Updating regression model with %d total data points...\n', length(collected_data.y));
                regression_model.train(collected_data.gamma, collected_data.y);
                
             
                if isempty(regression_model.beta)
                    fprintf('Regression model training failed. Performing random exploration.\n');
                    do_random_exploration = true;
                else
                    fprintf('Regression model updated successfully.\n');
                    do_random_exploration = (mod(i, params.c_rand) == 0);
                end
            
            if do_random_exploration
                fprintf('Performing random exploration...\n');
            else
                fprintf('Finding optimal gamma...\n');
            end
            
           
            gamma_scalar = findOptimalGamma(regression_model, params.delta, ...
                params.gamma_min, params.gamma_max, do_random_exploration);
            
            
            gamma_new = gamma_scalar * ones(params.N, 1);
            
           
            mpc.updateGamma(gamma_new);
            fprintf('Updated gamma: %.4f\n', gamma_scalar);
            
         
            gamma_history(i+1) = gamma_scalar;
            
            
            if ~isempty(regression_model.beta)
                [prob, ~] = regression_model.predict(gamma_scalar);
                constraint_prob(i+1) = prob;
                fprintf('Predicted constraint satisfaction probability: %.4f (target: >= %.4f)\n', ...
                    prob, 1-params.delta);
            else
             
                constraint_prob(i+1) = 0.5;
                fprintf('No regression model available. Using default probability: 0.5\n');
            end
        catch e
      
            fprintf('Error updating GP model: %s\n', e.message);
            gamma_history(i+1) = current_gamma;
            constraint_prob(i+1) = constraint_prob(i);
            
         
            if current_gamma < 0.5
                gamma_scalar = current_gamma + 0.05;
                gamma_new = gamma_scalar * ones(params.N, 1);
                mpc.updateGamma(gamma_new);
                gamma_history(i+1) = gamma_scalar;
                fprintf('Error recovery: Increased gamma to %.4f\n', gamma_scalar);
            end
        end
        
   
        infeasible_count = 0;
        feasible_count = 0;
        
        fprintf('---------------------------\n\n');
    end
    

    iteration(t+1) = i;
end

fprintf('Simulation completed.\n');

%% Prepare results
fprintf('Preparing results...\n');
results = struct();
results.time = time;
results.state = state;
results.input = input;
results.constraint_values = constraint_values;
results.iteration = 0:params.I;
results.gamma_history = gamma_history;
results.constraint_prob = constraint_prob;
results.delta = params.delta;
results.Q = params.Q;
results.R = params.R;

%% Evaluate performance
fprintf('Evaluating performance...\n');
performance = evaluatePerformance(results);

%% Plot results
fprintf('Plotting results...\n');
plotResults(results);

%% Final gamma value
fprintf('\nFinal results:\n');
fprintf('Final gamma value: %.4f\n', gamma_history(end));
fprintf('Final constraint satisfaction probability: %.4f (target: %.4f)\n', ...
    constraint_prob(end), 1-params.delta);

%% Save results
fprintf('Saving results to results.mat...\n');
save('results.mat', 'results', 'performance', 'params');
fprintf('Done.\n'); 
