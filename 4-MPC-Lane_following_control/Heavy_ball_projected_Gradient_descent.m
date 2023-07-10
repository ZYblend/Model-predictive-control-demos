function x = Heavy_ball_projected_Gradient_descent(x0,M,F,b,H1,H2,b1,b2,alpha,beta,error_tol,iter_tol)
%% function G = Heavy_ball_projected_MPC(A,B,x0,Q,R,h)
% This algorithm is to solve the following problem:
%          Minimize (1/2)*x.'*M*x
%        Subject to F*x = b
%                |H1*x| <= b1
%                |H2*x| <= b2
%
% Formulate the lagrange equation:
%  f(x,lambda) = (1/2)*x.'*M*x + lambda.'*(H1*x-b1)
% 
% Algorithm:
%
% 
% Input:
%       x0 [n-by-1]: initial variable
%       M [n-by-n]: weight matrix 
%       H1[m1-by-n], b1[m1-by-1]: euqality constraints
%       H2[m2-by-n], b2[m1-by-1]: inequality constraints
%       error_tol [scalar]: tolerance for optimal solution update; end when ||x_{k+1}-x_{k}||_2 <= error_tol  (0.01)
%       iter_tol [scalar]: tolerance for iteration steps; end when iter>iter_tol (100)
% Output:
%        x_hat [n-by-1]: optimal solution
%        lambda_hat [m1-by-1]: optimal lagrange multiplier
% Parameter:
%        alpha1 [n-by-1]: initial step size for x
%        alpha2 [m1-by-1]: initial step size for lambda
%        beta1 [n-by-1]: heavy ball multiplier for x 
%        beta2 [n-by-1]: heavy ball multiplier for lambda 
%
% Yu Zheng, Raslab, Florida State University, 2022
%
%% parameters
% gradient fucntion
Delta_fx = @(x)M*x;

% initialization
x_last = zeros(size(M,1),1);
x = x0;
iter = 1;

% define 
cProjFun = cell(5,1);
cProjFun{1} = @(y) all(F*y-b==0) * y + any(F*y-b~=0) *  (y-F.'*inv(F*F.')*(F*y-b));
cProjFun{2} = @(y) all(H1*y-b1<=0) * y + any(H1*y-b1>0) *  (y-H1.'*inv(H1*H1.')*(H1*y-b1));
cProjFun{3} = @(y) all(-H1*y-b1<=0) * y + any(-H1*y-b1>0) *  (y-H1.'*inv(H1*H1.')*(-H1*y-b1));
cProjFun{4} = @(y) all(H2*y-b2<=0) * y + any(H2*y-b2>0) *  (y-H2.'*inv(H2*H2.')*(H2*y-b2));
cProjFun{5} = @(y) all(-H2*y-b2<=0) * y + any(-H2*y-b2>0) *  (y-H2.'*inv(H2*H2.')*(-H2*y-b2));

while iter <= iter_tol
    %% heavy ball gradient descent
    x_ = x - alpha*Delta_fx(x) + beta*(x - x_last);
%     disp(x_)
     
    %% projection
    %%%%%%%%%%%%%%%%%%% projections on convex sets %%%%%%%%%%%%%%%%%%%%
    x_update = Orthogonal_proj_onto_multiConvexSet(cProjFun, x_, 100, 0.001);
    
    if all(F*x_update-b<=error_tol) && all(abs(H1*x_update)-b1<=error_tol) && all(abs(H2*x_update)-b2<=error_tol)
        disp('projection correct')
    else
        disp('projection wrong')
    end
    
    if iter == iter_tol
        disp('optimal solution not found but algorithm end');
    end
    iter = iter +1;
    
    %% update caches
%     disp(x_update)
    x_last = x;
    x = x_update;
    
    
    if Delta_fx(x) <= error_tol
        disp('optimal soultion found');
        break;
    end      
end
    
