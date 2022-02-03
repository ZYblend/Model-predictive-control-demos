function G = optimal_regulation_controller(A,B,x0,Q,R,h)
%% function G = optimal_tracking_controller(A,B,x0,Q,R,h)
% This function is to solve following optimal control problem:
%                       Minimize (1/2)*[ sum_{0~h}x.'*Q*x + sum_{0~h-1}u.'*Q*u ]
%                     Subject to x_{k+1} = A x_{k} + B u_{k}
%                                x_{0} = x0
% Once the above problem is solved, x converge to zero vector
%
% Inputs: 
%        - A [n-by-n], B [n-by-m]:  system discrete dynamics; (A,B) should be fully controllable
%        - x0 [n-by-1]           :  initial state vector
%        - Q [n-by-n] ,R [m-by-m]:  Q is state tracking error penlaty, R is control energy penlaty
% Outputs:
%        - G   : feedback gain
% Parameters:
%        - h: time horizon for optimal control
% Controller:
%        - u_k = G * x_{k-1}
%
% Example:
%          shown in "Optimal_tracking_control_sim.m"
%
% Yu Zheng, RASlab, Tallahassee, 2022/01/28
%
% Created during 2022 Spring course 'Intro to Model Predictive control', instructor: Dr. Olugbenga Moses Anubi
%
%% check controllability 
% unob = rank(ctrb(A,B))-length(A); 
% if unob == 0
%     fprintf('fully controllable\n');
% else
%     fprintf('not fully controllable\n');
%     keyboard
% end

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

F = [A_bar -B_bar;
     E0 zeros(n,m*h)];
 
b = [zeros(n*h,1);
     x0];

% Optimal Solution
temp = [M F.'; 
        F zeros(size(F,1))];
  
% check existence of unique solution
P = temp*inv(temp.'*temp)*temp.';
if abs(P*[zeros(n*(h+1)+m*h,1);b] - [zeros(n*(h+1)+m*h,1);b]) < 1e-5
    disp('Optimal solution is feasible');
else
    disp('Optimal solution is feasible, please check!');
%     keyboard
end
  
N = pinv(temp,0.01);

N12 = N(1:n*(h+1)+m*h,n*(h+1)+m*h+1:end);
    
z_star = N12*b;

% control gains
Nu = N12(n*(h+1)+1:end,end-n+1:end);

G = Nu(1:m,:);  % G is the first gain element in Nu 
                % Reason: Think in MPC way 
                % at each time step, we could see the state is x0, and then the G gain is just the first gain in Nu
                % which can make the control be optimal in h time horizon.
% disp('Optimal control gain G =');
% disp(num2str(G));