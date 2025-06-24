function gp_model = updateGPModel(gp_model, gamma, y_data)
% UPDATEGPMODEL Update the GP model with new data
%
% Inputs:
%   gp_model - GPBinaryRegression object
%   gamma - Constraint tightening parameter used
%   y_data - Binary labels (0/1) indicating constraint satisfaction
%
% Output:
%   gp_model - Updated GPBinaryRegression object

% Ensure y_data is a double precision column vector
y_data = double(y_data(:));

% Ensure gamma is a double precision matrix
gamma = double(gamma);

% Train the GP model directly with the data
gp_model.train(gamma, y_data);

end 