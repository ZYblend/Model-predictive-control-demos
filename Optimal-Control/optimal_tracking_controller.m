function [G,G_ff] = optimal_tracking_controller(A,B,x0,Q,R,x_ref,h)
%% function [G,G_ff] = optimal_tracking_controller(A,B,x0,Q,R,xr)
% This function is to solve following optimal control problem:
%                       Minimize (1/2)*[ sum_{0~h}(x-x_ref).'*Q*(x-x_ref) + sum_{0~h-1}u.'*Q*u ]
%                     Subject to x_{k+1} = A x_{k} + B u_{k}
%                                x_{0} = x0
%                                x_{h} = x_ref
% Once the above problem is solved, x could track x_ref
%
% Inputs: 
%        - A [n-by-n], B [n-by-m]:  system discrete dynamics; (A,B) should be fully controllable
%        - x0 [n-by-1]           :  initial state vector
%        - Q [n-by-n] ,R [m-by-m]:  Q is state tracking error penlaty, R is control energy penlaty
%        - x_ref [n-by-1]           :   reference state vector
% Outputs:
%        - G   : feedback gain
%        - G_ff: feedforward term
% Parameters:
%        - h: time horizon for optimal control
% Controller:
%        - u_k = G(x_{k-1}-x_ref) + G_ff
%
% Example:
%          shown in "Optimal_tracking_control_sim.m"
%
% Yu Zheng, RASlab, Tallahassee, 2022/01/28
%
% Created during 2022 Spring course 'Intro to Model Predictive control', instructor: Dr. Olugbenga Moses Anubi
%
%% check controllability 
unob = rank(ctrb(A,B))-length(A); 
if unob == 0
    fprintf('fully controllable\n');
else
    fprintf('not fully controllable\n');
    keyboard
end

%% dimensions
[n,m] = size(B);

%% Solver
% vectorization of the objective function
Q_bar = kron(eye(h+1),Q);
R_bar = kron(eye(h),R);
M = blkdiag(Q_bar, R_bar);

% vectorization of the constraints
A_bar1 = kron(eye(h),-A) ;
A_bar2 = zeros(n*h,n);
A_bar = [A_bar1,A_bar2];
for iter = 1:h
    A_bar(n*(iter-1)+1:iter*n,iter*n+1:n*(iter+1)) = eye(n);
end
B_bar = kron(eye(h),B);
E0 = [eye(n), zeros(n,n*h)];
E1 = [zeros(n,n*h), eye(n)];

br = kron(ones(h,1),(A-eye(n))*x_ref);

F = [A_bar -B_bar;
     E0 zeros(n,m*h);
     E1 zeros(n,m*h)];
% F = [A_bar -B_bar;
%     E0 zeros(n,m*h)];
 
b = [br;
     x0-x_ref;
     zeros(n,1)];
% b = [br;
%      x0-x_ref];

% Optimal Solution
temp = [M F.'; 
        F zeros(size(F,1))];

% check existence of unique solution
P = temp*inv(temp.'*temp)*temp.';
if abs(P*[zeros(n*(h+1)+m*h,1);b] - [zeros(n*(h+1)+m*h,1);b]) < 1e-5
    disp('Optimal solution is feasible');
else
    disp('Optimal solution is feasible, please check!');
    keyboard
end
  
N = pinv(temp,0.01);
N12 = N(1:n*(h+1)+m*h,n*(h+1)+m*h+1:end);
z_star = N12*b;

% control gains
Nb = N12(n*(h+1)+1:end,1:n*h);
Nu = N12(n*(h+1)+1:end,end-2*n+1:end-n);

% G is the first gain element in Nu 
                 % Reason: Think in MPC way 
                 % at each time step, we could see the state is x0, and then the G gain is just the first gain in Nu
                 % which can make the control be optimal in h time horizon.

G = Nu(1:m,:);            % feedback gain
G2 = Nb(1:m,:);           % feedforward gain
G_ff = G2*br;      % feedforward term

disp('Optimal control feedback gain G =');
disp(num2str(G));
disp('Optimal control feedforward gain G2 =');
disp(num2str(G2));