# ADMM-QP-Solver

This solver is developed to solve Quadratic Programming (QP) problems using the Alternating Direction Method of Multipliers (ADMM). It is currently focused on solving QP problems found in Control Barrier Functions (CBF).

## Reference
The method implemented in this project is based on the paper "Optimal parameter selection for the alternating direction method of multipliers (ADMM): quadratic problems", available at [arXiv](https://arxiv.org/pdf/1306.2454).

## Environment

This version of the solver has been developed and tested in MATLAB and is compatible with MATLAB 2023a and later versions starting from MATLAB 2013a.

## Contributing

Contributions are welcome! Please feel free to submit code via Pull Requests or report issues and suggestions through GitHub Issues.

## License

This project is licensed under the GPL-3.0 License. For more details, please see the LICENSE file.

## Quick Start

Clone this repository to your local machine:
```bash
git clone https://github.com/your-username/ADMM-QP-Solver.git
```

Run test.m, which includes six different test cases.

## Function Usage

QUADPROG_ADMM Solves quadratic programming problems with ADMM.

Syntax:
x = quadprog_admm(H, f, A, b, lb, ub, x0)

Inputs:
H - Quadratic coefficient matrix (symmetric, positive definite)

f - Linear coefficient vector

A - Linear inequality constraint matrix (Ax <= b)

b - Right-hand side vector for inequality constraints

lb - Lower bounds for the variables

ub - Upper bounds for the variables

x0 - Initial guess for the variables

Output:
x - Solution vector
