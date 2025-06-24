classdef TraditionalMPC < handle
    properties
        A           % System matrix
        B           % Input matrix
        Q           % State cost matrix
        R           % Input cost matrix
        P           % Terminal cost matrix
        N           % Prediction horizon
        nx          % State dimension
        nu          % Input dimension
        u_min       % Input lower bound
        u_max       % Input upper bound
        h_constraint % State constraint function
        c_sl        % Slack variable penalty
    end
    
    methods
        function obj = TraditionalMPC(params)
            % Constructor for traditional MPC controller
            obj.A = params.A;
            obj.B = params.B;
            obj.Q = params.Q;
            obj.R = params.R;
            obj.N = params.N;
            obj.nx = params.nx;
            obj.nu = params.nu;
            obj.u_min = params.u_min;
            obj.u_max = params.u_max;
            obj.h_constraint = params.h_constraint;
            obj.c_sl = params.c_sl;
            
            % Compute terminal cost matrix using discrete Lyapunov equation
            obj.P = dlyap(obj.A', obj.Q);
            fprintf('Traditional MPC controller initialized.\n');
        end
        
        function [u, info] = solve(obj, x)
            % Solve the MPC optimization problem using quadprog directly
            
            % System dimensions
            nx = obj.nx;
            nu = obj.nu;
            N = obj.N;
            
            % System matrices
            A = obj.A;
            B = obj.B;
            
            % Number of decision variables: u_0, u_1, ..., u_{N-1}, slack
            n_u = N * nu;
            n_z = n_u + 1;  % +1 for slack variable
            
            % Precompute powers of A for prediction
            A_powers = cell(N+1, 1);
            A_powers{1} = eye(nx);
            for k = 1:N
                A_powers{k+1} = A * A_powers{k};
            end
            
            % Compute state prediction matrices
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
            
            % Initialize cost matrices
            H = zeros(n_z, n_z);
            f = zeros(n_z, 1);
            
            % State cost contribution
            for k = 1:N
                % Get the part of Gamma that corresponds to the inputs we're optimizing
                Gamma_k = Gamma{k};
                
                % State cost: x_k' * Q * x_k
                if k < N
                    Q_block = Gamma_k' * obj.Q * Gamma_k;
                    q_block = Gamma_k' * obj.Q * Phi{k} * x;
                else
                    % Terminal cost: x_N' * P * x_N
                    Q_block = Gamma_k' * obj.P * Gamma_k;
                    q_block = Gamma_k' * obj.P * Phi{k} * x;
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
            H(n_u+1, n_u+1) = obj.c_sl;
            
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
            
            % 2. State constraints: h(x_k) <= 0 + slack
            A_x = zeros(N, n_z);
            b_x = zeros(N, 1);
            
            for k = 1:N
                % Extract the first row of Gamma_k (effect on x_1)
                Gamma_k_row1 = Gamma{k}(1, :);
                
                % Extract the first element of Phi{k}*x_0 (effect of initial state on x_1)
                Phi_k_x0 = Phi{k} * x;
                Phi_k_x0_1 = Phi_k_x0(1);  % Get the first element
                
                % Set up the constraint: [1 0] * x_k <= slack
                % This becomes: Gamma_k_row1 * u <= -Phi_k_x0_1 + slack
                
                % Input part
                A_x(k, 1:length(Gamma_k_row1)) = Gamma_k_row1;
                
                % Slack part
                A_x(k, n_u + 1) = -1;  % -slack
                
                % Right-hand side
                b_x(k) = -Phi_k_x0_1;
            end
            
            % 3. Non-negativity of slack variable: slack >= 0
            A_s = zeros(1, n_z);
            b_s = 0;
            
            A_s(1, n_u + 1) = -1;  % -slack <= 0
            
            % Combine all constraints
            A_ineq = [A_u; A_x; A_s];
            b_ineq = [b_u; b_x; b_s];
            
            % Solve QP with quadprog
            options = optimset('Display', 'off');
            [z, fval, exitflag, output] = quadprog(H, f, A_ineq, b_ineq, [], [], [], [], [], options);
            
            % Extract solution
            if exitflag > 0
                u = z(1:nu);  % Extract first control input
                slack_value = z(end);  % Extract slack variable
            else
                u = zeros(nu, 1);  % Default to zero input
                slack_value = 0;
            end
            
            % Prepare additional info
            info = struct();
            info.status = exitflag;
            info.objective = fval;
            info.slack = slack_value;
            info.predicted_states = [];  % Not computed for simplicity
            info.predicted_inputs = [];  % Not computed for simplicity
            
            % Check if the problem was solved successfully
            if exitflag <= 0
                warning('Traditional MPC optimization problem was not solved successfully. Status: %d', exitflag);
            end
        end
    end
end 