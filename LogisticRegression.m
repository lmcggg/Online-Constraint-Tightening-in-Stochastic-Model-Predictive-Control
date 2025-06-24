classdef LogisticRegression < handle
    properties
        beta      
        degree   
        mu       
        sigma    
    end
    
    methods
        function obj = LogisticRegression(params)
          
            if isfield(params, 'poly_degree')
                obj.degree = params.poly_degree;
            else
                obj.degree = 3; 
            end
            obj.beta = [];
            obj.mu = [];
            obj.sigma = [];
        end
        
        function Phi = designMatrix(obj, X, is_training)
            
           
            if nargin < 3
                is_training = false;
            end
            
            X = X(:); 
            
           
            raw_features = zeros(length(X), obj.degree);
            for i = 1:obj.degree
                raw_features(:, i) = X.^i;
            end

            
            if is_training
                
                obj.mu = mean(raw_features, 1);
                obj.sigma = std(raw_features, 1);
                
                obj.sigma(obj.sigma < 1e-6) = 1;
            end
            
           
            if ~isempty(obj.mu)
                scaled_features = (raw_features - obj.mu) ./ obj.sigma;
            else
                scaled_features = raw_features; 
            end
            
          
            Phi = [ones(length(X), 1), scaled_features];
        end

        function train(obj, X, y)
         
            if isempty(X) || length(y) < 2
                warning('Not enough data. Skipping training.');
                obj.beta = [];
                return;
            end
            if length(unique(y)) < 2
                warning('Not enough data diversity. Skipping training.');
                obj.beta = [];
                return;
            end

            fprintf('Training Logistic Regression with degree %d polynomial...\n', obj.degree);
            
           
            Phi = obj.designMatrix(X, true); 
            
          
            cond_num = cond(Phi);
            if cond_num > 1e10
                fprintf('Warning: Design matrix is highly ill-conditioned (cond = %.2e). Adding regularization.\n', cond_num);
            end
            
            try
                
                opts = statset('MaxIter', 300, 'TolFun', 1e-6);
                
              
                [B, FitInfo] = lassoglm(Phi, y(:), 'binomial', 'Alpha', 0.001, ...
                                       'Lambda', 0.01, 'Standardize', false, ...
                                       'Options', opts);
                
               
                obj.beta = [FitInfo.Intercept; B];
                fprintf('Logistic Regression trained successfully with regularization.\n');
            catch e
                warning('Error with lassoglm, falling back to glmfit: %s', e.message);
                try
                   
                    opts = statset('MaxIter', 300, 'TolFun', 1e-6);
                    [obj.beta, ~, ~] = glmfit(Phi, y(:), 'binomial', 'link', 'logit', 'options', opts);
                    fprintf('Logistic Regression trained with glmfit.\n');
                catch e2
                    warning('Error training Logistic Regression: %s', e2.message);
                  
                    obj.beta = zeros(size(Phi, 2), 1);
                    obj.beta(1) = 0;
                end
            end
        end
        
        function [prob, score_std] = predict(obj, X_test)
        
            if isempty(obj.beta)
                warning('Model not trained. Returning default probability 0.5.');
                prob = 0.5 * ones(size(X_test, 1), 1);
                score_std = ones(size(X_test, 1), 1);
                return;
            end
            
            
            Phi_test = obj.designMatrix(X_test, false);
            
          
            y_pred = glmval(obj.beta, Phi_test, 'logit');
            
            prob = y_pred;
            score_std = zeros(size(prob)); 
        end
    end
end 