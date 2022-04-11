function u_opt = constrained_MPC_pgdhb(x0,u0,M,F,H1,H2,b1,b2,h,n,m)
%% function u_opt = constrained_MPC_quaprog(x0,u0,M,F,H,b,h,n,m)
% constrained MPC based on projected heavy ball gradient descent

b = [zeros(n*h,1);
    x0];


Z0 = [kron(ones(h+1,1),x0);
      kron(ones(h,1),u0)];

alpha = 0.01;
beta = 0.001;

error_tol = 0.001;
iter_tol = 100;
opt = Heavy_ball_projected_Gradient_descent(Z0,M,F,b,H1,H2,b1,b2,alpha,beta,error_tol,iter_tol);

u_opt = opt(n*(h+1)+1:n*(h+1)+m);

end