function plotResults(results)
% PLOTRESULTS Plot simulation results


try
   
    figure('Name', 'Simulation Results', 'Position', [100, 100, 1200, 800]);

    
    subplot(3, 2, 1);
    plot(results.time, results.state(1, :), 'b-', 'LineWidth', 1.5);
    hold on;
    plot(results.time, results.state(2, :), 'r-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time step');
    ylabel('State');
    title('State Trajectory');
    legend('x_1', 'x_2');

   
    subplot(3, 2, 2);
    plot(results.time(1:end-1), results.input, 'k-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time step');
    ylabel('Control input');
    title('Control Inputs');

   
    subplot(3, 2, 3);
    constraint_values = results.constraint_values;
    plot(results.time, constraint_values, 'b-', 'LineWidth', 1.5);
    hold on;
    plot(results.time, zeros(size(results.time)), 'r--', 'LineWidth', 1);
    grid on;
    xlabel('Time step');
    ylabel('h(x)');
    title('Constraint Values (h(x) <= 0)');
    legend('h(x)', 'Constraint boundary');

   
    subplot(3, 2, 4);
   
    if length(results.iteration) ~= length(results.constraint_prob)
        warning('Iteration and constraint_prob have different lengths. Adjusting...');
        min_len = min(length(results.iteration), length(results.constraint_prob));
        iteration = results.iteration(1:min_len);
        constraint_prob = results.constraint_prob(1:min_len);
    else
        iteration = results.iteration;
        constraint_prob = results.constraint_prob;
    end
    
    plot(iteration, constraint_prob, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 6);
    hold on;
    plot(iteration, (1-results.delta)*ones(size(iteration)), 'r--', 'LineWidth', 1);
    grid on;
    xlabel('Iteration');
    ylabel('Probability');
    title('Constraint Satisfaction Probability');
    legend('Pr(h(x) <= 0)', 'Required (1-\delta)');

   
    subplot(3, 2, 5);
   
    if length(results.iteration) ~= length(results.gamma_history)
        warning('Iteration and gamma_history have different lengths. Adjusting...');
        min_len = min(length(results.iteration), length(results.gamma_history));
        iteration = results.iteration(1:min_len);
        gamma_history = results.gamma_history(1:min_len);
    else
        iteration = results.iteration;
        gamma_history = results.gamma_history;
    end
    
    plot(iteration, gamma_history, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 6);
    grid on;
    xlabel('Iteration');
    ylabel('\gamma');
    title('Constraint Tightening Parameter \gamma');

   
    subplot(3, 2, 6);
   
    x1_line = [0, 0];
    x2_line = [-2, 2];  

    plot(x1_line, x2_line, 'r--', 'LineWidth', 1.5);
    hold on;
    plot(results.state(1, :), results.state(2, :), 'b-', 'LineWidth', 1);
    plot(results.state(1, 1), results.state(2, 1), 'go', 'MarkerSize', 10, 'LineWidth', 2);  % Start point
    plot(results.state(1, end), results.state(2, end), 'ro', 'MarkerSize', 10, 'LineWidth', 2);  % End point
    grid on;
    xlabel('x_1');
    ylabel('x_2');
    title('State Trajectory in Phase Space');
    legend('Constraint Boundary (x_1 = 0)', 'Trajectory', 'Start', 'End');

    
    set(gcf, 'Color', 'w');
    sgtitle('Online Constraint Tightening in Stochastic MPC - DC-DC Converter');
catch e
    warning('Error in plotResults: %s', e.message);
    fprintf('Error details: %s\n', getReport(e));
end

end 