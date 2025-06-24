function performance = evaluatePerformance(results)
% EVALUATEPERFORMANCE Evaluate the performance of the controller


try
  
    performance = struct();

    
    constraint_violations = results.constraint_values > 0;
    performance.constraint_violation_probability = mean(constraint_violations);
    performance.constraint_satisfaction_probability = 1 - performance.constraint_violation_probability;

   
    state_cost = 0;
    input_cost = 0;
    total_cost = 0;

    for t = 1:length(results.time)-1
       
        state_cost = state_cost + results.state(:, t)' * results.Q * results.state(:, t);
        
        
        input_cost = input_cost + results.input(:, t)' * results.R * results.input(:, t);
        
       
        total_cost = total_cost + state_cost + input_cost;
    end

    
    performance.state_cost = state_cost / (length(results.time) - 1);
    performance.input_cost = input_cost / (length(results.time) - 1);
    performance.total_cost = total_cost / (length(results.time) - 1);

    if ~isempty(results.gamma_history)
        performance.final_gamma = results.gamma_history(end);
    else
        performance.final_gamma = NaN;
        warning('gamma_history is empty');
    end

   
    if length(results.gamma_history) > 10
     
        last_gammas = results.gamma_history(end-9:end);
        performance.gamma_std = std(last_gammas);
        
      
        performance.gamma_converged = performance.gamma_std < 0.01;
    else
        performance.gamma_converged = false;
        performance.gamma_std = NaN;
    end

  
    iterations = unique(results.iteration);
    performance.violations_per_iteration = zeros(size(iterations));

    for i = 1:length(iterations)
        iter = iterations(i);
        idx = results.iteration == iter;
        if any(idx)
            performance.violations_per_iteration(i) = mean(constraint_violations(idx));
        else
            performance.violations_per_iteration(i) = NaN;
        end
    end

  
    fprintf('===== Performance Summary =====\n');
    fprintf('Constraint satisfaction probability: %.4f (target: %.4f)\n', ...
        performance.constraint_satisfaction_probability, 1-results.delta);
    fprintf('Average state cost: %.4f\n', performance.state_cost);
    fprintf('Average input cost: %.4f\n', performance.input_cost);
    fprintf('Average total cost: %.4f\n', performance.total_cost);
    fprintf('Final gamma value: %.4f\n', performance.final_gamma);
    if performance.gamma_converged
        fprintf('Gamma has converged (std: %.4f)\n', performance.gamma_std);
    else
        fprintf('Gamma has not converged yet\n');
    end
    fprintf('==============================\n');
catch e
    warning('Error in evaluatePerformance: %s', e.message);
    fprintf('Error details: %s\n', getReport(e));
    performance = struct('error', e.message);
end

end 