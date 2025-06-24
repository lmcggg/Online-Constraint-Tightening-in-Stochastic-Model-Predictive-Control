classdef GPBinaryRegression < handle
    % GPBinaryRegression Class for Gaussian Process binary regression
    % Used to learn the mapping from gamma to constraint satisfaction probability
    
    properties
        gp_model            % Trained GP model
        kernel_type         % Type of kernel function
        sigmoid_type        % Type of sigmoid function (link function)
        optimize_hyperparams % Whether to optimize hyperparameters
    end
    
    methods
        function obj = GPBinaryRegression(params)
            % Constructor for GPBinaryRegression
            % Input:
            %   params - Parameter struct
            
            % Initialize properties
            obj.kernel_type = params.kernel_type;
            obj.sigmoid_type = params.sigmoid_type;
            obj.optimize_hyperparams = params.optimize_hyperparams;
        end
        
        function train(obj, X, y)
            % 检查数据
            if isempty(X) || length(y) < 2
                warning('Not enough data. Skipping GP training.');
                obj.gp_model = []; % Reset model
                return;
            end
            
            % Check if all data belongs to one class
            if length(unique(y)) < 2
                warning('Not enough data diversity to train. Skipping GP training.');
                obj.gp_model = []; % Reset model
                return;
            end
            
            % 确保 y 是 logical, fitgpc 推荐
            y_logical = (y == 1);
            
            fprintf('Training GP classifier with %d data points...\n', length(y));
            
            try
                % 使用 fitgpc 进行高斯过程二元分类
                % 'KernelFunction' -> 'squaredexponential' (即RBF)
                % 'Link' -> 'probit' (对应高斯误差函数) or 'logit'
                obj.gp_model = fitgpc(X, y_logical, ...
                    'KernelFunction', obj.kernel_type, ...
                    'Link', obj.sigmoid_type, ...
                    'FitMethod', 'exact', ...
                    'PredictMethod', 'exact');
                
                % 如果需要优化超参数
                if obj.optimize_hyperparams
                    fprintf('Optimizing hyperparameters...\n');
                    obj.gp_model = fitgpc(X, y_logical, ...
                        'KernelFunction', obj.kernel_type, ...
                        'Link', obj.sigmoid_type, ...
                        'OptimizeHyperparameters', 'auto', ...
                        'HyperparameterOptimizationOptions', struct('ShowPlots', false, 'Verbose', 0));
                end
                
                fprintf('GP model trained successfully.\n');
                
            catch e
                warning('Error training GP model: %s', e.message);
                obj.gp_model = []; % Reset model on failure
            end
        end
        
        function [prob, score_std] = predict(obj, gamma)
            % 检查模型
            if isempty(obj.gp_model)
                warning('GP model is not trained. Returning default probability 0.5.');
                prob = 0.5 * ones(size(gamma, 1), 1);
                score_std = ones(size(gamma, 1), 1);
                return;
            end
            
            gamma = double(gamma(:));
            
            % predict 返回 [label, posterior_probabilities, scores]
            % posterior_probabilities 的第二列是 P(y=1)
            [~, posterior, scores] = predict(obj.gp_model, gamma);
            
            prob = posterior(:, 2);
            
            % fitgpc不直接提供潜函数方差，但我们可以用scores的范围近似不确定性
            % 这里返回scores的标准差作为不确定性的一个代理指标
            score_std = scores(:,2); % 返回第二类的分数
        end
    end
end 