clc, clear, close all
% Testing the quadprog_admm function

%% Test Case 1: Small-scale unconstrained problem

H = [2, 0; 0, 2];
f = [-2; -5];
A = [];
b = [];
lb = [];
ub = [];

x0 = [0; 0];
% Timing quadprog_admm
tic;
[x_admm, info_admm] = quadprog_admm(H, f, A, b, lb, ub, x0);
time_admm = toc;
% Timing MATLAB's quadprog function
options = optimoptions('quadprog', 'Display', 'off');
tic;
[x_quadprog, fval_quadprog, exitflag_quadprog] = quadprog(H, f, A, b, [], [], lb, ub, x0, options);
time_quadprog = toc;

% Displaying results
disp('**************Test Case 1: Small-scale unconstrained problem**************');
fprintf('ADMM Solution: [%f, %f], Function Value: %f, Exit Flag: %d, Iterations: %d, Time: %.4f seconds\n', x_admm, info_admm.val, info_admm.exitflag, info_admm.iter, time_admm);
fprintf('Quadprog Solution: [%f, %f], Function Value: %f, Exit Flag: %d, Time: %.4f seconds\n', x_quadprog, fval_quadprog, exitflag_quadprog, time_quadprog);

% Comparing solutions
tolerance = 1e-6;
difference = norm(x_admm - x_quadprog);

if difference < tolerance
    disp('(√)The solutions from ADMM and quadprog are sufficiently close.');
else
    disp('(x)The solutions differ significantly!');
    fprintf('Difference norm: %f\n', difference);
end


%% Test Case 2: Small-scale bounded problem

H = [4, 1; 1, 3];
f = [1; 2];
A = [];
b = [];
lb = [0; 0];
ub = [10; 10];


x0 = [2; 2];

% Timing quadprog_admm
tic;
[x_admm, info_admm] = quadprog_admm(H, f, A, b, lb, ub, x0);
time_admm = toc;

% Timing MATLAB's quadprog function
options = optimoptions('quadprog', 'Display', 'off');
tic;
[x_quadprog, fval_quadprog, exitflag_quadprog] = quadprog(H, f, A, b, [], [], lb, ub, x0, options);
time_quadprog = toc;

% Displaying results
disp('**************Test Case 2: Small-scale bounded problem**************');
fprintf('ADMM Solution: [%f, %f], Function Value: %f, Exit Flag: %d, Iterations: %d, Time: %.4f seconds\n', x_admm, info_admm.val, info_admm.exitflag, info_admm.iter, time_admm);
fprintf('Quadprog Solution: [%f, %f], Function Value: %f, Exit Flag: %d, Time: %.4f seconds\n', x_quadprog, fval_quadprog, exitflag_quadprog, time_quadprog);

% Comparing solutions 
tolerance = 1e-6;
difference = norm(x_admm - x_quadprog);

if difference < tolerance
    disp('(√)The solutions from ADMM and quadprog are sufficiently close.');
else
    disp('(x)The solutions differ significantly!');
    fprintf('Difference norm: %f\n', difference);
end


%% Test Case 3: Large-scale problem with linear constraints

H = diag(rand(10,1) * 10);
f = rand(10,1) * 5;
A = rand(5,10);
b = A * ones(10,1);
lb = [];
ub = [];

x0 = zeros(10,1);

% Timing quadprog_admm
tic;
[x_admm, info_admm] = quadprog_admm(H, f, A, b, lb, ub, x0);
time_admm = toc;

% Timing MATLAB's quadprog function
options = optimoptions('quadprog', 'Display', 'off');
tic;
[x_quadprog, fval_quadprog, exitflag_quadprog] = quadprog(H, f, A, b, [], [], lb, ub, x0, options);
time_quadprog = toc;

% Displaying results
disp('**************Test Case 3: Large-scale problem with linear constraints**************');
fprintf('ADMM Solution: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f], Function Value: %f, Exit Flag: %d, Iterations: %d, Time: %.4f seconds\n', x_admm, info_admm.val, info_admm.exitflag, info_admm.iter, time_admm);
fprintf('Quadprog Solution: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f], Function Value: %f, Exit Flag: %d, Time: %.4f seconds\n', x_quadprog, fval_quadprog, exitflag_quadprog, time_quadprog);

% Comparing solutions
tolerance = 1e-6;
difference = norm(x_admm - x_quadprog);

if difference < tolerance
    disp('(√)The solutions from ADMM and quadprog are sufficiently close.');
else
    disp('(x)The solutions differ significantly!');
    fprintf('Difference norm: %f\n', difference);
end


%% Test Case 4: Positive definite matrix with a single inequality constraint

H = [3, 2; 2, 6];
f = [-2; -5];
A = [1, 2];
b = [5];
lb = [];
ub = [];

% 初始解
x0 = [0; 0];

% Timing quadprog_admm
tic;
[x_admm, info_admm] = quadprog_admm(H, f, A, b, lb, ub, x0);
time_admm = toc;

% Timing MATLAB's quadprog function
options = optimoptions('quadprog', 'Display', 'off');
tic;
[x_quadprog, fval_quadprog, exitflag_quadprog] = quadprog(H, f, A, b, [], [], lb, ub, x0, options);
time_quadprog = toc;

% Displaying results
disp('**************Test Case 4: Positive definite matrix with a single inequality constraint**************');
fprintf('ADMM Solution: [%f, %f], Function Value: %f, Exit Flag: %d, Iterations: %d, Time: %.4f seconds\n', x_admm, info_admm.val, info_admm.exitflag, info_admm.iter, time_admm);
fprintf('Quadprog Solution: [%f, %f], Function Value: %f, Exit Flag: %d, Time: %.4f seconds\n', x_quadprog, fval_quadprog, exitflag_quadprog, time_quadprog);

% Comparing solutions
tolerance = 1e-6;
difference = norm(x_admm - x_quadprog);

if difference < tolerance
    disp('(√)The solutions from ADMM and quadprog are sufficiently close.');
else
    disp('(x)The solutions differ significantly!');
    fprintf('Difference norm: %f\n', difference);
end


%% Test Case 5: Multiple bound constraints and no linear constraints

H = eye(3);
f = [1; 2; 3];
A = [];
b = [];
lb = [0; 0; 0];
ub = [10; 10; 10];


x0 = [3; 3; 3];

% Timing quadprog_admm
tic;
[x_admm, info_admm] = quadprog_admm(H, f, A, b, lb, ub, x0);
time_admm = toc;

% Timing MATLAB's quadprog function
options = optimoptions('quadprog', 'Display', 'off');
tic;
[x_quadprog, fval_quadprog, exitflag_quadprog] = quadprog(H, f, A, b, [], [], lb, ub, x0, options);
time_quadprog = toc;

% Displaying results
disp('**************Test Case 5: Multiple bound constraints and no linear constraints**************');
fprintf('ADMM Solution: [%f, %f, %f], Function Value: %f, Exit Flag: %d, Iterations: %d, Time: %.4f seconds\n', x_admm, info_admm.val, info_admm.exitflag, info_admm.iter, time_admm);
fprintf('Quadprog Solution: [%f, %f, %f], Function Value: %f, Exit Flag: %d, Time: %.4f seconds\n', x_quadprog, fval_quadprog, exitflag_quadprog, time_quadprog);

% Comparing solutions
tolerance = 1e-6;
difference = norm(x_admm - x_quadprog);

if difference < tolerance
    disp('(√)The solutions from ADMM and quadprog are sufficiently close.');
else
    disp('(x)The solutions differ significantly!');
    fprintf('Difference norm: %f\n', difference);
end


%% Test Case 6: Large-scale problem with random data

n = 20;
H = rand(n);
H = H'*H + eye(n)*0.1; 
f = rand(n, 1)*10 - 5;
A = rand(10, n)*2 - 1;
b = A * ones(n, 1);
lb = [];
ub = [];

x0 = zeros(n, 1);

% Timing quadprog_admm
tic;
[x_admm, info_admm] = quadprog_admm(H, f, A, b, lb, ub, x0);
time_admm = toc;

% Timing MATLAB's quadprog function
options = optimoptions('quadprog', 'Display', 'off');
tic;
[x_quadprog, fval_quadprog, exitflag_quadprog] = quadprog(H, f, A, b, [], [], lb, ub, x0, options);
time_quadprog = toc;

% Displaying results
disp('**************Test Case 6: Large-scale problem with random data**************');
fprintf('ADMM Solution: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], Function Value: %f, Exit Flag: %d, Iterations: %d, Time: %.4f seconds\n', x_admm, info_admm.val, info_admm.exitflag, info_admm.iter, time_admm);
fprintf('Quadprog Solution: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], Function Value: %f, Exit Flag: %d, Time: %.4f seconds\n', x_quadprog, fval_quadprog, exitflag_quadprog, time_quadprog);

% Comparing solutions
tolerance = 1e-6;
difference = norm(x_admm - x_quadprog);

if difference < tolerance
    disp('(√)The solutions from ADMM and quadprog are sufficiently close.');
else
    disp('(x)The solutions differ significantly!');
    fprintf('Difference norm: %f\n', difference);
end