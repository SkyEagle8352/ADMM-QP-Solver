function [x, info] = quadprog_admm(H, f, A, b, lb, ub, x0)
% QUADPROG_ADMM Solves quadratic programming problems using ADMM
%
% Syntax:
% [x, info] = quadprog_admm(H, f, A, b, lb, ub, x0)
%
% Inputs:
% H - Quadratic coefficient matrix (symmetric, positive definite)
% f - Linear coefficient vector
% A - Linear inequality constraint matrix (Ax <= b)
% b - Right hand side vector for inequality constraints
% lb - Lower bounds for the variables
% ub - Upper bounds for the variables
% x0 - Initial guess for the variables
%
% Outputs:
% x - Solution vector
% info - Structure containing the following details:
%    .val - Objective function value at the solution
%    .exitflag - Exit status (0: success, 1: max iterations, 2: error)
%    .iter - Number of iterations performed

    n = size(H, 1); 
    if nargin >= 5 && ~isempty(lb) && ~isempty(ub)
        nl_cons = length(lb);
        nu_cons = length(ub);
        Au = [eye(nu_cons); -eye(nl_cons)];
        bu = [ub; -lb];
        A_aug = [Au; A];
        b_aug = [bu; b];
    else
        A_aug = A;
        b_aug = b;
    end
    
    % Algorithm parameters
    max_iter = 5000;
    tol_iter = 1e-9;
    rho = 1;


    % Initialize variables
    if nargin < 7 || isempty(x0)
        x0 = zeros(n, 1);
    end
    x = x0; 
    info.exitflag = 2;
    info.iter = 0;

    % Handle empty constraints
    if isempty(A_aug) || isempty(b_aug)
        if isempty(A_aug) && isempty(b_aug)
            x = -H\f; % Direct solve for unconstrained problem
            info.val = 0.5*x'*H*x + f'*x;
            info.exitflag = 0;
            info.iter = 1;
        else
            info.exitflag = 2;
            info.val = NaN;
            error('Dimension of A and b must be consistent.');
        end
        return
    end

    % Precompute the inverse matrix used in the iterations
    invMat = (H + rho * (A_aug' * A_aug))\eye(n);
    z = zeros(size(b_aug));
    u = z;

    % Main ADMM iteration loop
    for iter = 1:max_iter
        x_new = -invMat * (f + rho * A_aug' * (z + u - b_aug));
        z_new = max(0, -A_aug * x_new - u + b_aug);
        u_new = u + A_aug * x_new - b_aug + z_new;

        e_tol = norm(x_new - x, 2) + norm(z_new - z, 2) + norm(u_new - u, 2);

        info.val = 0.5 * x' * H * x + f' * x;
        if e_tol <= tol_iter
            x = x_new;
            info.exitflag = 0;
            info.iter = iter;
            break;
        end

        x = x_new;
        z = z_new;
        u = u_new;

        if iter >= max_iter
            info.exitflag = 1;
            info.val = 0.5 * x' * H * x + f' * x;
            info.iter = iter;
            warning('Iteration has reached the maximum iterations.');
            break;
        end
    end
end
