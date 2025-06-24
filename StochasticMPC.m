classdef StochasticMPC < handle
    % StochasticMPC Class for Stochastic Model Predictive Control with 
    % constraint tightening
    
    properties
        system      % Linear system model for prediction
        N           % Prediction horizon
        Q           % State cost matrix
        R           % Input cost matrix
        P           % Terminal cost matrix
        c_sl        % Slack variable penalty
        u_min       % Lower bound on input
        u_max       % Upper bound on input
        h_constraint % Constraint function: h(x) <= 0
        gamma       % Constraint tightening parameter
    end
    
    methods
        function obj = StochasticMPC(system, params)
            % Constructor for StochasticMPC
            % Inputs:
            %   system - LinearSystem object
            %   params - Parameter struct
            
            obj.system = system;
            obj.N = params.N;
            obj.Q = params.Q;
            obj.R = params.R;
            obj.P = params.P;
            obj.c_sl = params.c_sl;
            obj.u_min = params.u_min;
            obj.u_max = params.u_max;
            obj.h_constraint = params.h_constraint;
            obj.gamma = params.gamma_init;
        end
        
        function [u, p_t, info] = solve(obj, x_t)
            % Solve the MPC problem using a condensed QP formulation
            % Inputs:
            %   x_t - Current state
            % Outputs:
            %   u - Optimal control input to apply
            %   p_t - Number of constraints relaxed
            %   info - Additional information about the solution
            
            % System dimensions and parameters
            nx = obj.system.nx;
            nu = obj.system.nu;
            N = obj.N;
            
            % Initialize variables
            info = struct('exitflag', -1, 'message', 'Infeasible', 'slack', 0);
            p_t = 0;
            u = zeros(nu, 1);  % Default to zero input
            slack_value = 0;
            
            % Backup strategy: find minimum p_t for feasibility
            while info.exitflag < 1 && p_t <= N
                try
                    % Number of decision variables: u_0, u_1, ..., u_{N-1}, s_1, ..., s_{p_t}
                    n_u = N * nu;
                    n_s = p_t;
                    n_z = n_u + n_s;
                    
                    % Compute matrices for the condensed QP
                    [H, f, A_ineq, b_ineq] = obj.formCondensedQP(x_t, p_t);
                    
                    % Solve QP with quadprog
                    options = optimset('Display', 'off');
                    [z, fval, exitflag, output] = quadprog(H, f, A_ineq, b_ineq, [], [], [], [], [], options);
                    
                    if exitflag > 0
                        % Solution found
                        u = z(1:nu);  % Extract first control input
                        
                        % Extract slack variables (if any)
                        if p_t > 0
                            slack_values = z(n_u+1:end);
                            slack_value = sum(slack_values);  % Sum of all slack variables
                        else
                            slack_value = 0;
                        end
                        
                        info.exitflag = exitflag;
                        info.message = 'Optimal solution found';
                        info.slack = slack_value;
                        info.objective = fval;
                        return;
                    else
                        % No solution found, increase p_t
                        p_t = p_t + 1;
                    end
                catch e
                    % Error in optimization, print message and increase p_t
                    fprintf('Error in optimization with %d slack variables: %s\n', p_t, e.message);
                    p_t = p_t + 1;
                end
            end
            
            % If still not feasible with maximum relaxation
            warning('MPC problem infeasible even with p_t = %d. Applying zero input.', N);
            u = zeros(nu, 1);
            p_t = N;
            info.exitflag = -1;
            info.message = 'Infeasible after all relaxations.';
            info.slack = 0;
        end
        
        function [H, f, A_ineq, b_ineq] = formCondensedQP(obj, x_0, p_t)
            % Form the condensed QP problem
            % Inputs:
            %   x_0 - Initial state
            %   p_t - Number of constraints to relax
            % Outputs:
            %   H, f - Quadratic cost matrices
            %   A_ineq, b_ineq - Inequality constraints
            
            % System dimensions
            nx = obj.system.nx;
            nu = obj.system.nu;
            N = obj.N;
            
            % System matrices
            A = obj.system.A;
            B = obj.system.B;
            
            % Number of decision variables
            n_u = N * nu;
            n_s = p_t;
            n_z = n_u + n_s;
            
            % Initialize cost matrices
            H = zeros(n_z, n_z);
            f = zeros(n_z, 1);
            
            % Precompute powers of A for prediction
            A_powers = cell(N+1, 1);
            A_powers{1} = eye(nx);
            for k = 1:N
                A_powers{k+1} = A * A_powers{k};
            end
            
            % Compute state prediction matrices
            % x_k = A^k * x_0 + [A^{k-1}*B, A^{k-2}*B, ..., B] * [u_0; u_1; ...; u_{k-1}]
            Phi = cell(N, 1);  % State transition matrices
            Gamma = cell(N, 1);  % Input effect matrices
            
            for k = 1:N
                % Phi_k = A^k
                Phi{k} = A_powers{k+1};
                
                % Gamma_k = [A^{k-1}*B, A^{k-2}*B, ..., B]
                Gamma{k} = zeros(nx, k*nu);
                for j = 1:k
                    col_idx = (j-1)*nu+1 : j*nu;
                    Gamma{k}(:, col_idx) = A_powers{k-j+1} * B;
                end
            end
            
            % Compute cost matrices for the condensed problem
            % J = 1/2 * u' * H * u + f' * u + constant
            
            % State cost contribution
            for k = 1:N
                % Get the part of Gamma that corresponds to the inputs we're optimizing
                Gamma_k = Gamma{k};
                
                % State cost: x_k' * Q * x_k
                if k < N
                    Q_block = Gamma_k' * obj.Q * Gamma_k;
                    q_block = Gamma_k' * obj.Q * Phi{k} * x_0;
                else
                    % Terminal cost: x_N' * P * x_N
                    Q_block = Gamma_k' * obj.P * Gamma_k;
                    q_block = Gamma_k' * obj.P * Phi{k} * x_0;
                end
                
                % Add to H and f
                rows = 1:size(Q_block, 1);
                cols = 1:size(Q_block, 2);
                H(rows, cols) = H(rows, cols) + Q_block;
                f(rows) = f(rows) + 2 * q_block;
            end
            
            % Input cost contribution
            for k = 1:N
                idx = (k-1)*nu+1 : k*nu;
                H(idx, idx) = H(idx, idx) + obj.R;
            end
            
            % Slack variable cost
            for k = 1:p_t
                idx = n_u + k;
                H(idx, idx) = obj.c_sl;
            end
            
            % Constraints
            % 1. Input constraints: u_min <= u_k <= u_max
            A_u = zeros(2*n_u, n_z);
            b_u = zeros(2*n_u, 1);
            
            % Upper bounds: u_k <= u_max
            for k = 1:N
                idx = (k-1)*nu+1 : k*nu;
                A_u(idx, idx) = eye(nu);
                b_u(idx) = obj.u_max * ones(nu, 1);
            end
            
            % Lower bounds: -u_k <= -u_min
            for k = 1:N
                idx_u = (k-1)*nu+1 : k*nu;
                idx_c = n_u + (k-1)*nu+1 : n_u + k*nu;
                A_u(idx_c, idx_u) = -eye(nu);
                b_u(idx_c) = -obj.u_min * ones(nu, 1);
            end
            
            % 2. State constraints: h(x_k) <= 0
            % For the DC-DC converter, h(x) = x_1
            % We need to express this in terms of u
            A_x = zeros(N, n_z);
            b_x = zeros(N, 1);
            
            for k = 1:N
                % Predict x_k = A^k * x_0 + Gamma_k * u_{0:k-1}
                % For the constraint x_1 <= 0, we need the first row of the prediction
                
                % Extract the first row of Gamma_k (effect on x_1)
                Gamma_k_row1 = Gamma{k}(1, :);
                
                % Extract the first element of Phi{k}*x_0 (effect of initial state on x_1)
                Phi_k_x0 = Phi{k} * x_0;
                Phi_k_x0_1 = Phi_k_x0(1);  % Get the first element
                
                % Set up the constraint: [1 0] * x_k <= -gamma_k + s_k
                % This becomes: Gamma_k_row1 * u <= -gamma_k - Phi_k_x0_1 + s_k
                
                % Input part
                A_x(k, 1:length(Gamma_k_row1)) = Gamma_k_row1;
                
                % Slack part (if this constraint can be relaxed)
                if k <= p_t
                    A_x(k, n_u + k) = -1;  % -s_k
                end
                
                % Right-hand side
                b_x(k) = -obj.gamma(k) - Phi_k_x0_1;
            end
            
            % 3. Non-negativity of slack variables: s_k >= 0
            A_s = zeros(p_t, n_z);
            b_s = zeros(p_t, 1);
            
            for k = 1:p_t
                A_s(k, n_u + k) = -1;  % -s_k <= 0
            end
            
            % Combine all constraints
            A_ineq = [A_u; A_x; A_s];
            b_ineq = [b_u; b_x; b_s];
        end
        
        function updateGamma(obj, gamma_new)
            % Update the constraint tightening parameter
            % Input:
            %   gamma_new - New constraint tightening parameter
            
            obj.gamma = gamma_new;
        end
    end
end 