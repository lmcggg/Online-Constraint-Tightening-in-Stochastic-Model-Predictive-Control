#  Stochastic Model Predictive Control with Online Constraint Tightening

This repository contains MATLAB implementation of a novel approach to Stochastic Model Predictive Control (SMPC) with online constraint tightening based on machine learning techniques. The framework uses regression models to learn the relationship between constraint tightening parameters and constraint satisfaction probability.
This repo is  inspired by "Online Constraint Tightening in Stochastic Model Predictive Control: A Regression Approach "-IEEE TRANSACTIONS ON AUTOMATIC CONTROL, VOL. 70, NO. 2, FEBRUARY 2025(Alexandre Capone , Student Member, IEEE, Tim Brüdigam , Member, IEEE,and Sandra Hirche , Fellow, IEEE),thanks for their paper.
## Key Features

- **Stochastic MPC**: Implementation of a stochastic MPC controller with constraint tightening
- **Online Learning**: Adaptive constraint tightening based on observed constraint violations
- **Multiple Regression Models**: Support for both Logistic Regression and Gaussian Process Binary Regression
- **Comparative Analysis**: Tools to compare traditional MPC with the proposed stochastic approach
- **Performance Evaluation**: Comprehensive performance metrics and visualization tools

## Project Structure

- **Main Scripts**
  - `main.m`: Main script to run the stochastic MPC simulation
  - `main_compare.m`: Script to compare traditional MPC with stochastic MPC
  - `config.m`: Configuration file with all system and controller parameters

- **System Models**
  - `system/LinearSystem.m`: Linear system dynamics implementation
  - `system/NonlinearSystem.m`: Nonlinear system dynamics with configurable nonlinearity

- **Control Algorithms**
  - `control/StochasticMPC.m`: Stochastic MPC controller with constraint tightening
  - `control/TraditionalMPC.m`: Traditional MPC controller for comparison

- **Learning Components**
  - `learning/LogisticRegression.m`: Logistic regression model for constraint satisfaction prediction
  - `learning/GPBinaryRegression.m`: Gaussian Process binary regression model
  - `learning/updateGPModel.m`: Function to update the GP model
  - `learning/findOptimalGamma.m`: Function to find the optimal tightening parameter

- **Utilities**
  - `utils/evaluatePerformance.m`: Functions to evaluate controller performance
  - `utils/plotResults.m`: Functions for visualization of results

## Algorithm Overview

The implemented approach follows these key steps:

1. Initialize the stochastic MPC controller with an initial constraint tightening parameter γ
2. Run the system for a collection period and observe constraint violations
3. Update the regression model using collected data
4. Find the optimal γ that ensures constraint satisfaction with probability ≥ 1-δ
5. Update the controller with the new γ value
6. Repeat steps 2-5 for multiple iterations

## Example Application: DC-DC Converter

The default configuration implements control of a DC-DC converter system with:
- 2-dimensional state space
- 1-dimensional input
- State constraint on the first state variable (x₁ ≤ 0)
- Uniform bounded disturbances

## Requirements

- MATLAB R2019b or newer
- Statistics and Machine Learning Toolbox
- Optimization Toolbox
- Control System Toolbox

## How to Run

1. Clone this repository
2. Open MATLAB and navigate to the project directory
3. Modify parameters in `config.m` if needed
4. Run `main.m` for stochastic MPC simulation
5. Run `main_compare.m` to compare with traditional MPC

## Results

The simulation results include:
- State and input trajectories
- Constraint violation statistics
- Evolution of the tightening parameter γ
- Predicted constraint satisfaction probability
- Performance metrics including control cost and constraint violations

![image](https://github.com/user-attachments/assets/06c603d4-a434-4cb2-a0f7-1194e317bc5f)
![image](https://github.com/user-attachments/assets/4c52628e-feaf-4d8a-8986-1671245a74a2)

![image](https://github.com/user-attachments/assets/e8cc5386-eea1-454e-b006-97f55ce070d8)

Stochastic MPC:
  Constraint violations: 435 out of 5500 steps (7.91%)
  Final gamma value: 0.1455
  Final constraint satisfaction probability: 0.9519 (target: 0.9500)

Traditional MPC:
  Constraint violations: 2394 out of 5500 steps (43.53%)
## License

This project is licensed under the MIT License - see the LICENSE file for details. 
