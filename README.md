# TransMPC: Stochastic Model Predictive Control with Online Constraint Tightening

This repository contains MATLAB implementation of a novel approach to Stochastic Model Predictive Control (SMPC) with online constraint tightening based on machine learning techniques. The framework uses regression models to learn the relationship between constraint tightening parameters and constraint satisfaction probability.

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

## Citation

If you use this code in your research, please cite:

```
@article{transmpc2023,
  title={Online Constraint Tightening in Stochastic Model Predictive Control: A Regression Approach},
  author={Your Name},
  journal={arXiv preprint},
  year={2023}
}
```

## License

This project is licensed under the MIT License - see the LICENSE file for details. 