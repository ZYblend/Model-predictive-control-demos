%% optimal tracking control simulation
% Description:
%             In this file, I simulate an optimal tracking control problem:
%                      Minimize (1/2)*[ sum_{0~h}(x-x_ref).'*Q*(x-x_ref) + sum_{0~h-1}u.'*Q*u ]
%                    Subject to  x_{k+1} = A x_{k} + B u_{k}
%                                x_{0} = x0
%                                x_{h} = x_ref
%
% Yu Zheng, RASlab, Tallahassee, 2022/01/28
%
% Used for 2022 Spring course 'Intro to Model Predictive control', instructor: Dr. Olugbenga Moses Anubi
%
clear all
clc

%% parameters
h = 10;   % total time steps
n = 10;   % state dimension
m = 10;   % input dimension
% [A,B]= sysGen(m,n);
% x0 = rand(n,1);
load A.mat
load B.mat
% load x0.mat
x0 = zeros(n,1);

Q = 5*eye(n);
R = 0.001*eye(m);

x_ref = linspace(1,n,n).'; % ones(n,1);

%% Optimal tracking control solver
[G,G_ff] = optimal_tracking_controller(A,B,x0,Q,R,x_ref,h);

%% Simulation
total_time = 20;
x1 = x0;          % for optimal control 1 simulaiton
x2 = x0;          % for lqr control simulaiton
X1 = zeros(n,total_time+1);  % cache, for plotting
X2 = zeros(n,total_time+1);  % cache, for plotting
for iter = 1:total_time+1
    % optimal control 1
    X1(:,iter) = x1;
    u1 = G*(x1-x_ref) + G_ff;
    x_next1 = A*x1 + B*u1;
    
    x1 = x_next1;
    
%     % lqr control
%     X2(:,iter) = x2; 
%     u2 = -G_lqr*(x2-x_ref);
%     x_next2 = A*x2 + B*u2;
%     
%     x2 = x_next2;  
end

% plotting
t = linspace(0,total_time,total_time+1);

figure (1)
plot(t,X1);
title('vecterization solver (optimal control)')

% figure (2)
% plot(t,X2);
% title('lqr solver (optimal control)')
    
    
