clc, clear, close all
%% 测试用例 1: 小规模无约束问题
% 定义H矩阵和f向量
H = [2, 0; 0, 2];
f = [-2; -5];

% 没有约束
A = [];
b = [];
lb = [];
ub = [];

% 初始解
x0 = [0; 0];

% 调用自定义函数并计时
tic;
[x_admm, info_admm] = quadprog_admm(H, f, A, b, lb, ub, x0);
time_admm = toc;

% 调用MATLAB quadprog函数并计时
options = optimoptions('quadprog', 'Display', 'off');
tic;
[x_quadprog, fval_quadprog, exitflag_quadprog] = quadprog(H, f, A, b, [], [], lb, ub, x0, options);
time_quadprog = toc;

% 输出结果
disp('Test Case 1: Small-scale unconstrained problem');
fprintf('ADMM Solution: [%f, %f], Function Value: %f, Exit Flag: %d, Iterations: %d, Time: %.4f seconds\n', x_admm, info_admm.val, info_admm.exitflag, info_admm.iter, time_admm);
fprintf('Quadprog Solution: [%f, %f], Function Value: %f, Exit Flag: %d, Time: %.4f seconds\n', x_quadprog, fval_quadprog, exitflag_quadprog, time_quadprog);

% 比较两个解是否足够接近
tolerance = 1e-6;
difference = norm(x_admm - x_quadprog);

if difference < tolerance
    disp('The solutions from ADMM and quadprog are sufficiently close.');
else
    disp('The solutions differ significantly.');
    fprintf('Difference norm: %f\n', difference);
end


%% 测试用例 2: 带有上下界的小规模问题
% 定义H矩阵和f向量
H = [4, 1; 1, 3];
f = [1; 2];

% 没有线性约束
A = [];
b = [];

% 定义界限
lb = [0; 0];
ub = [10; 10];

% 初始解
x0 = [2; 2];

% 调用自定义函数并计时
tic;
[x_admm, info_admm] = quadprog_admm(H, f, A, b, lb, ub, x0);
time_admm = toc;

% 调用MATLAB quadprog函数并计时
options = optimoptions('quadprog', 'Display', 'off');
tic;
[x_quadprog, fval_quadprog, exitflag_quadprog] = quadprog(H, f, A, b, [], [], lb, ub, x0, options);
time_quadprog = toc;

% 输出结果
disp('Test Case 2: Small-scale bounded problem');
fprintf('ADMM Solution: [%f, %f], Function Value: %f, Exit Flag: %d, Iterations: %d, Time: %.4f seconds\n', x_admm, info_admm.val, info_admm.exitflag, info_admm.iter, time_admm);
fprintf('Quadprog Solution: [%f, %f], Function Value: %f, Exit Flag: %d, Time: %.4f seconds\n', x_quadprog, fval_quadprog, exitflag_quadprog, time_quadprog);

% 比较两个解是否足够接近
tolerance = 1e-6;
difference = norm(x_admm - x_quadprog);

if difference < tolerance
    disp('The solutions from ADMM and quadprog are sufficiently close.');
else
    disp('The solutions differ significantly.');
    fprintf('Difference norm: %f\n', difference);
end


%% 测试用例 3: 大规模带线性约束的问题
% 大规模H矩阵和f向量
H = diag(rand(10,1) * 10);
f = rand(10,1) * 5;

% 定义线性约束
A = rand(5,10);
b = A * ones(10,1);

% 定义界限
lb = [];
ub = [];

% 初始解
x0 = zeros(10,1);

% 调用自定义函数并计时
tic;
[x_admm, info_admm] = quadprog_admm(H, f, A, b, lb, ub, x0);
time_admm = toc;

% 调用MATLAB quadprog函数并计时
options = optimoptions('quadprog', 'Display', 'off');
tic;
[x_quadprog, fval_quadprog, exitflag_quadprog] = quadprog(H, f, A, b, [], [], lb, ub, x0, options);
time_quadprog = toc;

% 输出结果
disp('Test Case 3: Large-scale problem with linear constraints');
fprintf('ADMM Solution: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f],, Function Value: %f, Exit Flag: %d, Iterations: %d, Time: %.4f seconds\n', x_admm, info_admm.val, info_admm.exitflag, info_admm.iter, time_admm);
fprintf('Quadprog Solution: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f],, Function Value: %f, Exit Flag: %d, Time: %.4f seconds\n', x_quadprog, fval_quadprog, exitflag_quadprog, time_quadprog);

% 比较两个解是否足够接近
tolerance = 1e-6;
difference = norm(x_admm - x_quadprog);

if difference < tolerance
    disp('The solutions from ADMM and quadprog are sufficiently close.');
else
    disp('The solutions differ significantly.');
    fprintf('Difference norm: %f\n', difference);
end


%% 测试用例 4: 正定矩阵和单个不等式约束
% 定义H矩阵和f向量
H = [3, 2; 2, 6];
f = [-2; -5];

% 定义线性约束
A = [1, 2];
b = [5];

% 无界限
lb = [];
ub = [];

% 初始解
x0 = [0; 0];

% 调用自定义函数并计时
tic;
[x_admm, info_admm] = quadprog_admm(H, f, A, b, lb, ub, x0);
time_admm = toc;

% 调用MATLAB quadprog函数并计时
options = optimoptions('quadprog', 'Display', 'off');
tic;
[x_quadprog, fval_quadprog, exitflag_quadprog] = quadprog(H, f, A, b, [], [], lb, ub, x0, options);
time_quadprog = toc;

% 输出结果
disp('Test Case 4: Positive definite matrix with a single inequality constraint');
fprintf('ADMM Solution: [%f, %f], Function Value: %f, Exit Flag: %d, Iterations: %d, Time: %.4f seconds\n', x_admm, info_admm.val, info_admm.exitflag, info_admm.iter, time_admm);
fprintf('Quadprog Solution: [%f, %f], Function Value: %f, Exit Flag: %d, Time: %.4f seconds\n', x_quadprog, fval_quadprog, exitflag_quadprog, time_quadprog);

% 比较两个解是否足够接近
tolerance = 1e-6;
difference = norm(x_admm - x_quadprog);

if difference < tolerance
    disp('The solutions from ADMM and quadprog are sufficiently close.');
else
    disp('The solutions differ significantly.');
    fprintf('Difference norm: %f\n', difference);
end


%% 测试用例 5: 多个界限约束和无线性约束
% 定义H矩阵和f向量
H = eye(3);
f = [1; 2; 3];

% 无线性约束
A = [];
b = [];

% 定义界限
lb = [0; 0; 0];
ub = [10; 10; 10];

% 初始解
x0 = [3; 3; 3];

% 调用自定义函数并计时
tic;
[x_admm, info_admm] = quadprog_admm(H, f, A, b, lb, ub, x0);
time_admm = toc;

% 调用MATLAB quadprog函数并计时
options = optimoptions('quadprog', 'Display', 'off');
tic;
[x_quadprog, fval_quadprog, exitflag_quadprog] = quadprog(H, f, A, b, [], [], lb, ub, x0, options);
time_quadprog = toc;

% 输出结果
disp('Test Case 5: Multiple bound constraints and no linear constraints');
fprintf('ADMM Solution: [%f, %f, %f], Function Value: %f, Exit Flag: %d, Iterations: %d, Time: %.4f seconds\n', x_admm, info_admm.val, info_admm.exitflag, info_admm.iter, time_admm);
fprintf('Quadprog Solution: [%f, %f, %f], Function Value: %f, Exit Flag: %d, Time: %.4f seconds\n', x_quadprog, fval_quadprog, exitflag_quadprog, time_quadprog);

% 比较两个解是否足够接近
tolerance = 1e-6;
difference = norm(x_admm - x_quadprog);

if difference < tolerance
    disp('The solutions from ADMM and quadprog are sufficiently close.');
else
    disp('The solutions differ significantly.');
    fprintf('Difference norm: %f\n', difference);
end


%% 测试用例 6: 大规模问题，随机数据
% 随机生成大规模H矩阵和f向量
n = 20;
H = rand(n);
H = H'*H + eye(n)*0.1;  % 确保H是正定的
f = rand(n, 1)*10 - 5;

% 随机生成线性约束
A = rand(10, n)*2 - 1;
b = A * ones(n, 1);

% 定义界限
lb = [];
ub = [];

% 初始解
x0 = zeros(n, 1);

% 调用自定义函数并计时
tic;
[x_admm, info_admm] = quadprog_admm(H, f, A, b, lb, ub, x0);
time_admm = toc;

% 调用MATLAB quadprog函数并计时
options = optimoptions('quadprog', 'Display', 'off');
tic;
[x_quadprog, fval_quadprog, exitflag_quadprog] = quadprog(H, f, A, b, [], [], lb, ub, x0, options);
time_quadprog = toc;

% 输出结果
disp('Test Case 6: Large-scale problem with random data');
fprintf('ADMM Solution: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], Function Value: %f, Exit Flag: %d, Iterations: %d, Time: %.4f seconds\n', x_admm, info_admm.val, info_admm.exitflag, info_admm.iter, time_admm);
fprintf('Quadprog Solution: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], Function Value: %f, Exit Flag: %d, Time: %.4f seconds\n', x_quadprog, fval_quadprog, exitflag_quadprog, time_quadprog);

% 比较两个解是否足够接近
tolerance = 1e-6;
difference = norm(x_admm - x_quadprog);

if difference < tolerance
    disp('The solutions from ADMM and quadprog are sufficiently close.');
else
    disp('The solutions differ significantly.');
    fprintf('Difference norm: %f\n', difference);
end