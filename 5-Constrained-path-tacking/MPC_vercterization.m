function [M,F,H_bar,H1,H2,b_bar,b1,b2] = MPC_vercterization(A,B,Q,R,h,tau1,tau2)
%% vectorization
[n,m] = size(B);

% vectorization of the objective function
Q_bar = kron(eye(h+1),Q);
R_bar = kron(eye(h),R);
M = blkdiag(Q_bar, R_bar);

A_bar1 = kron(eye(h),-A) ;
A_bar2 = zeros(n*h,n);
A_bar = [A_bar1,A_bar2];
for iter = 1:h
    A_bar(n*(iter-1)+1:iter*n,iter*n+1:n*(iter+1)) = eye(n);
end
B_bar = kron(eye(h),B);
E0 = [eye(n), zeros(n,n*h)];
F = [A_bar -B_bar;
     E0 zeros(n,m*h)];


I_bar_temp = -eye(m*h) + diag(ones(m*(h-1),1),1);
I_bar = I_bar_temp(1:end-m,:);
H1 = [zeros(m*(h-1),n*(h+1)), I_bar];
b1 = kron(ones(h-1,1),tau1);
H2 = [zeros(m*h,n*(h+1)), eye(m*h)];
b2 = kron(ones(h,1),tau2);
H = [H1;H2];
b = [b1;b2];

H_bar = [H;-H];
b_bar = [b;b];


