classdef NonlinearSystem < handle
    % NonlinearSystem Class representing a nonlinear discrete-time system
    % x_{t+1} = f(x_t, u_t, w_t)
    
    properties
        A               % Linear part: system matrix
        B               % Linear part: input matrix
        nx              % State dimension
        nu              % Input dimension
        nw              % Noise dimension
        nonlinear_factor % Strength of the nonlinearity
    end
    
    methods
        function obj = NonlinearSystem(A, B, nonlinear_factor)
            % Constructor for NonlinearSystem
            % Inputs:
            %   A - Linear part: system matrix
            %   B - Linear part: input matrix
            %   nonlinear_factor - Strength of the nonlinearity
            
            obj.A = A;
            obj.B = B;
            obj.nonlinear_factor = nonlinear_factor;
            
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
            
            % Linear part: x_linear = Ax_t + Bu_t
            x_linear = obj.A * x + obj.B * u;
            
            % Nonlinear part: add a nonlinear term based on the state
            % This is just an example nonlinearity; can be modified as needed
            nonlinear_term = obj.nonlinear_factor * [x(1)^2 * sin(x(2)); x(2)^2 * cos(x(1))];
            
            % Complete dynamics: x_{t+1} = f(x_t, u_t) + w_t
            x_next = x_linear + nonlinear_term + w;
        end
        
        function linearized = getLinearizedModel(obj, x_op, u_op)
            % Get a linearized model around an operating point
            % Inputs:
            %   x_op - Operating point for state
            %   u_op - Operating point for input
            % Output:
            %   linearized - LinearSystem object
            
            % For this implementation, we simply return the linear part
            % In a more complex implementation, we would compute the Jacobian
            linearized = LinearSystem(obj.A, obj.B);
        end
    end
end 