%% simulation to compare the results from three algorithms for equality constrainted program:
%     Minimizer (1/2)*x.'*M*x
%     Subject to F*x = b
%           1. by using matlab fucntion quadprog
%           2. formulate lagrange function, get closed-form solution by first-order condition
%           3. projected gradient descent
%           4.
% Yu Zheng, Florida State University, 2022.04.10
% 
clear all
clc

%% random matrix for quadratic minimization problem
n = 50;
d = 10*rand(n,1);                        % The diagonal values
M = diag(d);

m = 20; 
F = 50*rand(m,n);
b = 100*rand(m,1);

%% 1. quadprog
tic
x_quad = quadprog(M,[],[],[],F,b);
toc


%% 2. closed-form solution (lagrange)
tic
G = [M F.';
     F, zeros(size(F,1))];
b_bar = [zeros(size(M,1),1);
         b];
opt_lagrange_Cf = pinv(G)*b_bar;

x_lagrange_cf = opt_lagrange_Cf(1:n);
mu_lagrange_cf = opt_lagrange_Cf(n+1:m+n);
toc

%% 3. projected gradient descent
tic
Delta_x = @(x)M*x;
iter_tol = 100;
tol_error = 1e-3;
alpha = 0.1;
x = zeros(n,1);
iter = 0;
while iter <= iter_tol
    iter = iter +1;
    x_ = x - alpha*Delta_x(x);
    
    if any(F*x_ - b ~= 0)
        x_update = x_ - F.'*inv(F*F.')*(F*x_ -b);
    else
        x_update = x_;
    end
        
    if all(Delta_x(x_update) < tol_error)
        disp('converged');
        x = x_update;
        break;
    end
    x = x_update;    
end
x_GD = x;
toc

%% 4. Projected gradient descent (augmented lagrange)
tic 
Delta_x_aug = @(x,mu) M*x+ mu*F.'*(F*x-b);
Delta_mu_aug = @(x,mu) (1/2)*norm(F*x-b)^2;

alpha1 = 0.1;
alpha2 = 0.1;
x = zeros(n,1);
mu = 1;
iter = 0;
while iter <= iter_tol
    iter = iter +1;
    x_ = x - alpha*Delta_x_aug(x,mu);
    mu_ = mu - alpha*Delta_mu_aug(x,mu);
    
    if any(F*x_ - b ~= 0)
        x_update = x_ - F.'*inv(F*F.')*(F*x_ -b);
    else
        x_update = x_;
    end
    
    mu_update = (mu_>=1)*mu_ + (mu_<1)*1;
        
    if all(Delta_x(x_update) < tol_error)
        disp('converged');
        x = x_update;
        mu = mu_update;
        break;
    end
    x = x_update;
    mu = mu_update;
end
x_GD_aug = x;
mu_GD_aug = mu;

toc
