function [x] = quadprog_admm(H,f,A,b,lb,ub,x0)
% QUADPROG_ADMM Solves quadratic programming problems with ADMM
%
% Syntax:
% [x] = quadprog_admm(H, f, A, b, lb, ub, x0)
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
% Output:
% x - Solution vector

    n = size(H, 1); % 获取问题的维度
    nl_cons = length(lb);
    nu_cons = length(ub);

    max_iter = 10000;
    tol_iter = 1e-6;
    rho = 1;
    
    if nargin < 7
        x0 = zeros(n, 1); % 初始化变量 x
    end
    
    if nargin >= 5
        Au = [eye(nu_cons);-eye(nl_cons)];
        bu = [ub;-lb];
        A_aug = [Au;A];
        b_aug = [bu;b];
    else
        A_aug = A;
        b_aug = b;
    end
    
    [x, ~] = admm_qp(H, f, A_aug, b_aug, x0, rho, max_iter, tol_iter);
end

function [x, info] = admm_qp(H, f, A, b, x0, rho, max_iter, tol_iter)
    % ADMM_QP Solves quadratic programming problems using ADMM
    %
    % Syntax:
    % [x, info] = admm_qp(H, f, A, b, x0, rho, max_iter, tol_iter)
    %
    % Inputs:
    % H - Quadratic coefficient matrix (symmetric, positive definite)
    % f - Linear coefficient vector
    % A - Linear inequality constraint matrix
    % b - Right hand side vector for inequality constraints
    % x0 - Initial guess for the variables
    % rho - Penalty parameter for ADMM
    % max_iter - Maximum number of iterations
    % tol_iter - Tolerance for convergence
    %
    % Outputs:
    % x - Solution vector
    % info - Structure containing the following details:
    %    .val - Objective function value at the solution
    %    .exitflag - Exit status (0: success, 1: max iterations, 2: error)
    %    .iter - Number of iterations performed

    x = x0; % 初始化变量 x
    info.exitflag = 2;
    info.iter = 0;
    
    if isempty(A)||isempty(b)
        if isempty(A)&&isempty(b)
            x = -H\f;
            info.val = 0.5*x'*H*x+f'*x;
            info.exitflag = 0;
            info.iter = 1;
        else
            info.exitflag = 2;
            info.val = NaN;
            error('Dimension of A and b must be consistent.')
        end
        return
    end

    invMat = inv(H+rho*(A'*A));
    z = zeros(size(b));
    u = z;
    for iter = 1:max_iter

        x_new = -invMat*(f+rho*A'*(z+u-b));
        z_new = max(0,-A*x_new-u+b);
        u_new = u+A*x_new-b+z_new;
      
        e_tol = norm(x_new-x,2)+norm(z_new-z,2)+norm(u_new-u,2);
        
        info.val = 0.5*x'*H*x+f'*x;
        if e_tol<=tol_iter
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
            info.val = 0.5*x'*H*x+f'*x;
            info.iter = iter;
            warning('Iteration has reached the maximum iterations.')
            break;
        end
    end 
end