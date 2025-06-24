function gamma_opt = findOptimalGamma(model, delta, gamma_min, gamma_max, do_random_exploration)
% FINDOPTIMALGAMMA Find the optimal constraint tightening parameter
%
% Inputs:
%   model - LogisticRegression or GPBinaryRegression object
%   delta - Risk level (constraint violation probability)
%   gamma_min - Minimum value of gamma
%   gamma_max - Maximum value of gamma
%   do_random_exploration - Whether to do random exploration
%
% Output:
%   gamma_opt - Optimal constraint tightening parameter


if do_random_exploration
    gamma_opt = gamma_min + (gamma_max - gamma_min) * rand();
    fprintf('Random exploration: gamma = %.4f\n', gamma_opt);
    return;
end

% Grid search to find optimal gamma
n_grid = 100;  
gamma_grid = linspace(gamma_min, gamma_max, n_grid)';


try
    [probs, ~] = model.predict(gamma_grid);
    
    
    valid_idx = find(probs >= 1-delta);
    
    if isempty(valid_idx)
     
        warning('No gamma satisfies chance constraint. Exploring a more conservative value.');
       
        gamma_opt = (gamma_min + gamma_max)/2 + (gamma_max - gamma_min)/2 * rand();
        fprintf('Exploring a more conservative gamma: %.4f\n', gamma_opt);
    else

        gamma_opt = min(gamma_grid(valid_idx));
        fprintf('Found valid gamma: %.4f (prob=%.4f)\n', gamma_opt, probs(valid_idx(1)));
    end
catch e
    
    warning('Error in findOptimalGamma: %s. Using default gamma.', e.message);
    gamma_opt = 0.2;  
end

end 