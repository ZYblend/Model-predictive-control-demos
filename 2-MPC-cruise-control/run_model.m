%% Run file for MPC-based cruise control
% 
% Yu Zheng, Raslab, Florida State University, 2022/02/03
%
clear all
clc
%% Vehicle dynamics
m = 2.7;
Cd = 10;

%% Control design
Q = 2;              % error tracking penalty
R = 0.01;           % control energy penalty

Ts = 0.01;          % discrete time step
tot = 20;
h = linspace(1,tot,tot);             % horizon

v_star = 5;         % desired velocity
Fr = -Cd*v_star^2;  % feedforward term
v0 = 2;             % initialized velocity

A = 1-(2*Cd*Ts*v_star/m);  % dicrete linearization dynamics
B = Ts/m;                  % dicrete linearization dynamics

F_fb = zeros(tot,1);
G_lqr = zeros(tot,1);
for iter = 1:tot
    F_fb(iter) = optimal_regulation_controller(A,B,v0-v_star,Q,R,h(iter));  
                           % feedback gain
    [G_lqr(iter),S,e] = dlqr(A,B,Q,R,zeros(1,1));
end

%% plotting 
% see the mpc gain converges to the lqr gain when horizon increase
plot(h,F_fb);
hold on, plot(h,-G_lqr)
xlabel('horizon');
ylabel('Gain');


