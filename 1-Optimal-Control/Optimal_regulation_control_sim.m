%% optimal control Solvers
% Description:
%             In this file, I compare two optimal control solver:
%                                1) vectorization optimization
%                                2) LQR solver
% They both solve the following optimal control problem:
%                      Minimize (1/2)*[ sum_{0~h}x.'*Q*x + sum_{0~h-1}u.'*Q*u ]
%                    Subject to  x_{k+1} = A x_{k} + B u_{k}
%                                x_{0} = x0
% 
% Yu Zheng, RASlab, Florida State University, 2022/01/28
%
% Used for 2022 Spring course 'Intro to Model Predictive control', instructor: Dr. Olugbenga Moses Anubi
%
clear all
clc

%% system parameters
h = 20;   % total time steps
n = 10;   % state dimension
m = 10;   % input dimension
% [A,B]= sysGen(m,n);
% x0 = rand(n,1);
load A.mat
load B.mat
load x0.mat

Q = 2*eye(n);
R = 0.01* eye(m);

%% Vectorization solver
G = optimal_regulation_controller(A,B,x0,Q,R,h);
                
%% LQR
[G_lqr,S,e] = dlqr(A,B,Q,R,zeros(n,m));
disp('Optimal control gain -G_lqr =');
disp(num2str(-G_lqr));

disp('Optimal control gain difference G - (-G_lqr) =');
disp(num2str(G+G_lqr));

keyboard 
%% Simulation
total_time = 15;
x1 = x0;          % for optimal control 1 simulaiton
x2 = x0;          % for lqr control simulaiton
X1 = zeros(n,total_time+1);  % cache, for plotting
X2 = zeros(n,total_time+1);  % cache, for plotting
for iter = 1:total_time+1
    % optimal control 1
    X1(:,iter) = x1;
    
    u1 = G*x1;
    x_next1 = A*x1 + B*u1;
    
    x1 = x_next1;
    
    % lqr control
    X2(:,iter) = x2;
    
    u2 = -G_lqr*x2;
    x_next2 = A*x2 + B*u2;
    
    x2 = x_next2;       
end

% plotting
t = linspace(0,total_time,total_time+1);

figure (1)
plot(t,X1);
title('vecterization solver (optimal control)')

figure (2)
plot(t,X2);
title('lqr solver (optimal control)')