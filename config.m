% Configuration file for Online Constraint Tightening in SMPC


%% System parameters
% System dimensions
params.nx = 2;      % State dimension
params.nu = 1;      % Input dimension
params.nw = 2;      % Noise dimension

% Linear system matrices for DC-DC converter 
params.A = [1, 0.0075; -0.143, 0.996];
params.B = [4.798; 0.115];

% True nonlinear system parameters 
params.f_nonlinear = false;  % Use linear system for simulation as in the paper
params.nonlinear_factor = 0;  % Not used when f_nonlinear is false

% Noise parameters 
params.noise_mean = [0; 0];
params.noise_cov = 0.01 * eye(2);  
params.noise_uniform_bounds = [-0.14, 0.14]; 

%% Constraint parameters
% State constraints: h(x) <= 0
params.h_constraint = @(x) x(1);  % Constraint on the first state: x_1 <= 0

% Input constraints: u_min <= u <= u_max
params.u_min = -0.2; 
params.u_max = 0.2;   

% Risk level for chance constraint: Pr(h(x) <= 0) >= 1-delta
params.delta = 0.05; 

%% MPC parameters
% Prediction horizon
params.N = 10;

% Cost matrices 
params.Q = [1, 0; 0, 0];  
params.R = 1;             

% Terminal cost matrix will be computed in main.m using dlyap

% Slack variable penalty
params.c_sl = 1e8;  

% Initial constraint tightening parameter
params.gamma_init = 0.2 * ones(params.N, 1);  

% Constraint tightening parameter search space
params.gamma_min = -0.1; 
params.gamma_max = 0.8;   

%% Online learning algorithm parameters
% Number of updates
params.I = 20;  

% Waiting time before collecting data (to reach steady state)
params.T_wait = 500; 

% Collection time for each gamma value
params.T_col = 5000; 

% Random exploration frequency
params.c_rand = 100; 

%% GP model parameters
% Kernel function parameters
params.kernel_type = 'squaredexponential';
params.psi = 1;       % Amplitude parameter
params.lambda = 1;    % Length scale parameter

% Sigmoid function parameters
params.sigmoid_type = 'probit';  % Use probit for fitgpc

% GP fitting options
params.gp_method = 'exact';  % 'exact' or 'sparse'
params.optimize_hyperparams = true;  % Whether to optimize hyperparameters

%% Logistic Regression parameters
params.poly_degree = 3;  % Polynomial degree for basis functions

%% Simulation parameters
% Initial state
params.x0 = [0; 0]; 

% Simulation time
params.T_sim = params.I * (params.T_wait + params.T_col);

% Random seed for reproducibility
params.rng_seed = 42; 