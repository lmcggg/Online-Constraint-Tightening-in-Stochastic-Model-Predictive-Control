classdef LinearSystem < handle
    % LinearSystem Class representing a linear discrete-time system
    % x_{t+1} = Ax_t + Bu_t + w_t
    
    properties
        A       % System matrix
        B       % Input matrix
        nx      % State dimension
        nu      % Input dimension
        nw      % Noise dimension
    end
    
    methods
        function obj = LinearSystem(A, B)
            % Constructor for LinearSystem
            % Inputs:
            %   A - System matrix
            %   B - Input matrix
            
            obj.A = A;
            obj.B = B;
            
            % Determine dimensions
            obj.nx = size(A, 1);
            obj.nu = size(B, 2);
            obj.nw = obj.nx;  % Assuming noise dimension equals state dimension
        end
        
        function x_next = step(obj, x, u, w)
            % Simulate one step of the system dynamics
            % Inputs:
            %   x - Current state
            %   u - Control input
            %   w - Process noise (optional)
            % Output:
            %   x_next - Next state
            
            if nargin < 4
                w = zeros(obj.nw, 1);  % Default: no noise
            end
            
            % Linear dynamics: x_{t+1} = Ax_t + Bu_t + w_t
            x_next = obj.A * x + obj.B * u + w;
        end
        
        function [A_pred, B_pred] = getPredictionMatrices(obj, N)
            % Get matrices for N-step prediction
            % Input:
            %   N - Prediction horizon
            % Outputs:
            %   A_pred - Prediction matrix for autonomous dynamics
            %   B_pred - Prediction matrix for control inputs
            
            nx = obj.nx;
            nu = obj.nu;
            
            % Initialize matrices
            A_pred = zeros(nx * N, nx);
            B_pred = zeros(nx * N, nu * N);
            
            % First prediction step
            A_pred(1:nx, :) = obj.A;
            B_pred(1:nx, 1:nu) = obj.B;
            
            % Build prediction matrices
            for i = 2:N
                % Update A_pred
                A_pred((i-1)*nx+1:i*nx, :) = obj.A * A_pred((i-2)*nx+1:(i-1)*nx, :);
                
                % Update B_pred
                B_pred((i-1)*nx+1:i*nx, 1:nu) = obj.A * B_pred((i-2)*nx+1:(i-1)*nx, 1:nu);
                
                % Fill in the rest of B_pred for this row
                for j = 2:i
                    B_pred((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu) = ...
                        B_pred((i-j)*nx+1:(i-j+1)*nx, 1:nu);
                end
            end
        end
    end
end 