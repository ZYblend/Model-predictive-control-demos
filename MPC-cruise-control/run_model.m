%% Run file for MPC-based cruise control
% 
% Yu Zheng, Raslab, Florida State University, 2022/02/03
%
clear all
clc
%% Vehicle dynamics
m = 2.7;
Cd = 1000;

%% Control design
Q = 2;     % error tracking penalty
R = 0.01; % control energy penalty
Ts = 0.01;        % discrete time step
h = 10;  % horizon
v_star = 5;
Fr = -Cd*v_star^2;


%% simulation
v0 = 0;  % initialization



