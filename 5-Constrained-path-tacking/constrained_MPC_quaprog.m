function u_opt = constrained_MPC_quaprog(x0,u0,M,F,H,b,h,n,m)
%% function u_opt = constrained_MPC_quaprog(A,B,x0,Q,R,h,G,tau1,tau2,tau3)
% This function is to solve the following MPC problem by fmincon:
%      Minimize    (1/2)*[ sum_{0~h}x.'*Q*x + sum_{0~h-1}u.'*Q*u  + sum_{0~h-1}f*s^2]
%      Subject to   x_{k+1}    = A x_{k} + B u_{k} 
%                   x_{0}      = x0
%                   |G x_{k}-s| <= tau1  for all k =0,1,...,h  [box constraint for state]
%                   |u_{k}|   <= tau3 for all k =0,1,...,h-1   [box constraint for input]
%                   |u_{k} - u_{-1}| <= (1+k) tau2 * |u_{-1}|  [rate constraint for input]
%
% Inputs: 
%         A[n-by-n],B1[n-by-m]           : system (error) dynamics
%         z0[1-by-3-cell]                : {x0, u0, s0}
                                          % x0[n-by-1]: initial state
                                          % u0[m-by-1]: last-time control input
                                          % s0[q-by-1]: the box contraint error based on x0
%         Q[n-by-n], R[m-by-m], f[q-by-q]: Q is state tracking error penlaty, R is control energy penlaty, f is the penlty for error beyond box constraint of state
%         h [scalar]                     : horizon length
%         G [q-by-n],tau1[q-by-1]        : tau1 is the upper bound of the box containt, 
%                                          G = I or only has one element 1
%         tau2 [m-by-1]                  : rate contraints for control input
%         tau3 [m-by-1]                  : box contraints for control input
% Outputs:
%         u_opt [m-by-1]                 : optimal control input at current time step
%
% User tips:
%           1) derive your error dynamics, then use this program to regulate your error state to zero
%           2) Although, I only output u_opt which is the control input you
%           should use, you could also get the other optimal control
%           inputs, states and error of contraint opt=[x; u; s]
%
% Yu Zheng, RASlab, Tallahassee, 2022/03/30
%
%
g = [zeros(n*h,1);
    x0];


% opt = quadprog(M,[],H,b,F,g);
%%
Z0 = [kron(ones(h+1,1),x0);
      kron(ones(h,1),u0)];
fun = @(z)(1/2)*z.'*M*z;
opt = fmincon(fun,Z0,H,b,F,g);

u_opt = opt(n*(h+1)+1:n*(h+1)+m);

end



