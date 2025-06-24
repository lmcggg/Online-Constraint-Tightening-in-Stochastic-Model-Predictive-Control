%% Comparison of Traditional MPC and Stochastic MPC


%% Setup
clear;
clc;
close all;


addpath('system', 'control', 'learning', 'utils');


config;

rng(params.rng_seed);


params.I = 5;         
params.T_wait = 100;  
params.T_col = 1000;   
params.T_sim = params.I * (params.T_wait + params.T_col);

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

%% Initialize controllers
fprintf('Initializing controllers...\n');

smpc = StochasticMPC(linear_system, params);


tmpc = TraditionalMPC(params);

%% Initialize regression model for SMPC
fprintf('Initializing Logistic Regression model...\n');
regression_model = LogisticRegression(params);

%% Initialize simulation variables
fprintf('Initializing simulation variables...\n');

T_sim = params.T_sim;
time = 0:T_sim;


nx = params.nx;
nu = params.nu;


smpc_state = zeros(nx, T_sim + 1);
smpc_input = zeros(nu, T_sim);
smpc_state(:, 1) = params.x0;
smpc_constraint_values = zeros(1, T_sim + 1);
smpc_constraint_values(1) = params.h_constraint(smpc_state(:, 1));
smpc_slack = zeros(1, T_sim);

tmpc_state = zeros(nx, T_sim + 1);
tmpc_input = zeros(nu, T_sim);
tmpc_state(:, 1) = params.x0;
tmpc_constraint_values = zeros(1, T_sim + 1);
tmpc_constraint_values(1) = params.h_constraint(tmpc_state(:, 1));
tmpc_slack = zeros(1, T_sim);


smpc_violations = 0;
tmpc_violations = 0;


iteration = zeros(1, T_sim + 1);


gamma_history = zeros(1, params.I + 1); 
gamma_history(1) = params.gamma_init(1); 


constraint_prob = zeros(1, params.I + 1);  
constraint_prob(1) = 0.5; 


collected_data = struct('gamma', [], 'y', []);


smpc_infeasible_count = 0;
smpc_feasible_count = 0;
tmpc_infeasible_count = 0;
tmpc_feasible_count = 0;

%% Run simulation
fprintf('Starting simulation with %d iterations...\n', params.I);
fprintf('Each iteration: %d wait steps + %d collection steps = %d total steps\n', ...
    params.T_wait, params.T_col, params.T_wait + params.T_col);
fprintf('Total simulation steps: %d\n', T_sim);


i = 0;


noise_sequence = zeros(params.nw, T_sim);
for t = 1:T_sim
    noise_sequence(:, t) = params.noise_uniform_bounds(1) + ...
        (params.noise_uniform_bounds(2) - params.noise_uniform_bounds(1)) * rand(params.nw, 1);
end


for t = 1:T_sim
   
    if mod(t, ceil(T_sim/10)) == 0
        fprintf('Simulation progress: %.1f%% (step %d/%d)\n', 100*t/T_sim, t, T_sim);
    end
    
   
    smpc_x_t = smpc_state(:, t);
    tmpc_x_t = tmpc_state(:, t);
    
   
    [smpc_u_t, smpc_p_t, smpc_info] = smpc.solve(smpc_x_t);
    
    
    [tmpc_u_t, tmpc_info] = tmpc.solve(tmpc_x_t);
   
    if smpc_info.exitflag > 0
        smpc_feasible_count = smpc_feasible_count + 1;
    else
        smpc_infeasible_count = smpc_infeasible_count + 1;
    end
    
    if tmpc_info.status == 0
        tmpc_feasible_count = tmpc_feasible_count + 1;
    else
        tmpc_infeasible_count = tmpc_infeasible_count + 1;
    end
    

    smpc_input(:, t) = smpc_u_t;
    tmpc_input(:, t) = tmpc_u_t;
    
   
    w_t = noise_sequence(:, t);
    
    
    smpc_state(:, t+1) = true_system.step(smpc_x_t, smpc_u_t, w_t);
    tmpc_state(:, t+1) = true_system.step(tmpc_x_t, tmpc_u_t, w_t);
    
  
    smpc_constraint_values(t+1) = params.h_constraint(smpc_state(:, t+1));
    tmpc_constraint_values(t+1) = params.h_constraint(tmpc_state(:, t+1));
    
  s
    if smpc_constraint_values(t+1) > 0
        smpc_violations = smpc_violations + 1;
    end
    if tmpc_constraint_values(t+1) > 0
        tmpc_violations = tmpc_violations + 1;
    end
    

    if isfield(smpc_info, 'slack')
        smpc_slack(t) = smpc_info.slack;
    else
        smpc_slack(t) = 0;
    end
    
    if isfield(tmpc_info, 'slack')
        tmpc_slack(t) = tmpc_info.slack;
    else
        tmpc_slack(t) = 0;
    end
    
 
    iteration(t+1) = i;
 
    if mod(t, params.T_wait + params.T_col) == 0 && t > 0
        
        i = i + 1;
        
        fprintf('\n----- Iteration %d/%d -----\n', i, params.I);
        fprintf('SMPC Feasibility: %d feasible, %d infeasible (%.1f%% feasible)\n', ...
            smpc_feasible_count, smpc_infeasible_count, 100*smpc_feasible_count/(smpc_feasible_count+smpc_infeasible_count));
        fprintf('TMPC Feasibility: %d feasible, %d infeasible (%.1f%% feasible)\n', ...
            tmpc_feasible_count, tmpc_infeasible_count, 100*tmpc_feasible_count/(tmpc_feasible_count+tmpc_infeasible_count));
       
        collection_start = t - params.T_col + 1;
        collection_end = t;
        
        y_data = double(smpc_constraint_values(collection_start+1:collection_end+1) <= 0);
        
    
        n_satisfied = sum(y_data);
        n_violated = length(y_data) - n_satisfied;
        fprintf('SMPC Constraint: %d satisfied, %d violated (%.2f%% satisfied)\n', ...
            n_satisfied, n_violated, 100*n_satisfied/length(y_data));
        
        n_satisfied_tmpc = sum(tmpc_constraint_values(collection_start+1:collection_end+1) <= 0);
        n_violated_tmpc = params.T_col - n_satisfied_tmpc;
        fprintf('TMPC Constraint: %d satisfied, %d violated (%.2f%% satisfied)\n', ...
            n_satisfied_tmpc, n_violated_tmpc, 100*n_satisfied_tmpc/params.T_col);
        
      
        current_gamma = smpc.gamma(1);
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
            
         
            smpc.updateGamma(gamma_new);
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
           
            fprintf('Error updating model: %s\n', e.message);
            gamma_history(i+1) = current_gamma;
            constraint_prob(i+1) = constraint_prob(i);
            
      
            if current_gamma < 0.5
                gamma_scalar = current_gamma + 0.05;
                gamma_new = gamma_scalar * ones(params.N, 1);
                smpc.updateGamma(gamma_new);
                gamma_history(i+1) = gamma_scalar;
                fprintf('Error recovery: Increased gamma to %.4f\n', gamma_scalar);
            end
        end
        
     
        smpc_infeasible_count = 0;
        smpc_feasible_count = 0;
        tmpc_infeasible_count = 0;
        tmpc_feasible_count = 0;
        
        fprintf('---------------------------\n\n');
    end
    

    iteration(t+1) = i;
end

fprintf('Simulation completed.\n');

%% Prepare results
fprintf('Preparing results...\n');
results = struct();
results.time = time;

% SMPC results
results.smpc = struct();
results.smpc.state = smpc_state;
results.smpc.input = smpc_input;
results.smpc.constraint_values = smpc_constraint_values;
results.smpc.violations = smpc_violations;
results.smpc.violation_rate = smpc_violations / T_sim;
results.smpc.slack = smpc_slack;

% TMPC results
results.tmpc = struct();
results.tmpc.state = tmpc_state;
results.tmpc.input = tmpc_input;
results.tmpc.constraint_values = tmpc_constraint_values;
results.tmpc.violations = tmpc_violations;
results.tmpc.violation_rate = tmpc_violations / T_sim;
results.tmpc.slack = tmpc_slack;

% SMPC learning results
results.iteration = 0:params.I;
results.gamma_history = gamma_history;
results.constraint_prob = constraint_prob;
results.delta = params.delta;

%% Plot comparison results
fprintf('Plotting comparison results...\n');

% Plot states
figure('Name', 'State Comparison');
for i = 1:nx
    subplot(nx, 1, i);
    plot(time, smpc_state(i, :), 'b-', 'LineWidth', 1.5);
    hold on;
    plot(time, tmpc_state(i, :), 'r--', 'LineWidth', 1.5);
    ylabel(['x_' num2str(i)]);
    grid on;
    if i == 1
        title('State Comparison');
        legend('Stochastic MPC', 'Traditional MPC');
    end
end
xlabel('Time step');

% Plot inputs
figure('Name', 'Input Comparison');
stairs(time(1:end-1), smpc_input, 'b-', 'LineWidth', 1.5);
hold on;
stairs(time(1:end-1), tmpc_input, 'r--', 'LineWidth', 1.5);
ylabel('Control input (u)');
grid on;
title('Control Input Comparison');
legend('Stochastic MPC', 'Traditional MPC');
xlabel('Time step');

% Plot constraint values
figure('Name', 'Constraint Comparison');
plot(time, smpc_constraint_values, 'b-', 'LineWidth', 1.5);
hold on;
plot(time, tmpc_constraint_values, 'r--', 'LineWidth', 1.5);
plot(time, zeros(size(time)), 'k--');
ylabel('Constraint value h(x)');
grid on;
title('Constraint Value Comparison (h(x) ≤ 0 required)');
legend('Stochastic MPC', 'Traditional MPC', 'Constraint Boundary');
xlabel('Time step');

% Plot constraint violations
figure('Name', 'Constraint Violations');
subplot(2, 1, 1);
violations_smpc = smpc_constraint_values > 0;
violations_tmpc = tmpc_constraint_values > 0;
stem(time, violations_smpc, 'b');
hold on;
stem(time, violations_tmpc * 0.8, 'r');
ylabel('Violation');
title('Constraint Violations (1 = violated)');
legend('Stochastic MPC', 'Traditional MPC');
grid on;

subplot(2, 1, 2);
bar([results.smpc.violation_rate, results.tmpc.violation_rate]);
set(gca, 'XTickLabel', {'Stochastic MPC', 'Traditional MPC'});
ylabel('Violation Rate');
title(sprintf('Constraint Violation Rate (Target: ≤ %.2f)', params.delta));
grid on;

% Plot gamma history for SMPC
figure('Name', 'SMPC Learning');
subplot(2, 1, 1);
plot(results.iteration, gamma_history, 'bo-', 'LineWidth', 1.5);
ylabel('\gamma');
title('SMPC Constraint Tightening Parameter (\gamma) History');
grid on;

subplot(2, 1, 2);
plot(results.iteration, constraint_prob, 'bo-', 'LineWidth', 1.5);
hold on;
plot(results.iteration, (1-params.delta)*ones(size(results.iteration)), 'r--');
ylabel('Probability');
title('Predicted Constraint Satisfaction Probability');
legend('Predicted Probability', 'Target (1-\delta)');
grid on;
xlabel('Iteration');

%% Print final statistics
fprintf('\n--- Final Statistics ---\n');
fprintf('Stochastic MPC:\n');
fprintf('  Constraint violations: %d out of %d steps (%.2f%%)\n', ...
    results.smpc.violations, T_sim, 100*results.smpc.violation_rate);
fprintf('  Final gamma value: %.4f\n', gamma_history(end));
fprintf('  Final constraint satisfaction probability: %.4f (target: %.4f)\n', ...
    constraint_prob(end), 1-params.delta);

fprintf('\nTraditional MPC:\n');
fprintf('  Constraint violations: %d out of %d steps (%.2f%%)\n', ...
    results.tmpc.violations, T_sim, 100*results.tmpc.violation_rate);

%% Save comparison results
fprintf('\nSaving comparison results to results_compare.mat...\n');
save('results_compare.mat', 'results', 'params');
fprintf('Done.\n'); 