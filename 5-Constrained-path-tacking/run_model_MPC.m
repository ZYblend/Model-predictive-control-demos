%% Run file for MPC implementation
%% run model
clear all
clc

%% Model parameters
load Thetas.mat
L = 0.256;     % length of wheelbase (m)
Rw = 0.024;    % wheel radius (m)
rg = 9.49;     % total drive ratio


%% run system
% command satuation (for safety)
sat_pwm = 0.2;
sat_steering = pi/6;

% desired states
desired_longitudinal_speed = 2;                           % for cruise control
desired_motor_speed = 400 * desired_longitudinal_speed;   % 400 is the constant ratio between motor speed and the wheel speed

% desired trajectory (circle)
k_ref = 2;
r_ref =1;

% initial conditions
V0 = [0.0; 0.0; 0.0];      % [yaw rate; Vx; Vy]
X0 = [0.0; 2.0; 0.0];      % [x, y, heading]  
omegam0 = 0.0;
yaw0 = 0.0;
d0 = 0.1;  %0.8944;

Ts = 0.01;  % control frequency 1/Ts Hz

%% simplified model (longitudinal)
k1 = (1/2)*(Theta3+Theta4)*L^2;
k2 = 2*Theta5*Rw/rg;
k3 = 2*Theta6;
k4 = L*Theta5;
k5 = 2*Theta5;

K1 = Theta1*k5/(Theta2*k2);  % feedford gain for longitudinal controller


%% simplified model (discrete lateral error dynamics)
% 1. counter clockwise
Vx = desired_longitudinal_speed;
omegam = desired_motor_speed;
A_lat = [0 1 0 0;
         0 -2*Theta6 2*Theta6*Vx Theta5;
         0 0 0 1;
         0 0 0 -(L^2)*(Theta3+Theta4)/2];
B1_lat = [0; 
          Theta6*Vx+Theta5*Rw*omegam/rg; 
          0;
          Theta3*Vx*L/2 + Rw*L*Theta4*omegam/(2*rg)];
B2_lat = [0;
          Theta5-Vx;
          0;
          -(L^2)*(Theta3+Theta4)/2];

A_lat_d = eye(4) + A_lat*Ts;
B1_lat_d = B1_lat * Ts;
B2_lat_d = B2_lat * Ts;

% 2. clockwise
A_lat2 = [0 1 0 0;
         0 2*Theta6 -2*Theta6*Vx -Theta5;
         0 0 0 1;
         0 0 0 -(L^2)*(Theta3+Theta4)/2];
B1_lat2 = [0; 
          -Theta6*Vx-Theta5*Rw*omegam/rg; 
          0;
          Theta3*Vx*L/2 + Rw*L*Theta4*omegam/(2*rg)];
B2_lat2 = [0;
          -Theta5+Vx;
          0;
          -(L^2)*(Theta3+Theta4)/2];
      
A_lat_d2 = eye(4) + A_lat2*Ts;
B1_lat_d2 = B1_lat2 * Ts;
B2_lat_d2 = B2_lat2 * Ts;


%% trajectory
% 1. single polynomial
% Trajectory_generator;                    % p(1)*x.^2 + p(2)*x + p(3)
% x0d = 0.1;
% X0_star = [x0d; g(x0d); atan(g_dot(x0d))];  % initilization of trajectory

% 2. double polynominal
Trajectory_generator;           % p(1)*x.^2 + p(2)*x + p(3)
x0d = 0.1;
X0_star = [x0d; g1(x0d); atan(g_dot1(x0d))];     % initilization of trajectory 

%% error initialization
[e10,e20,~] = crosstrack_error(X0(1),X0(2),p1,X0(3));

% e10 = sqrt( (X0(1)-X0_star(1))^2 + (X0(2)-X0_star(2))^2 );  % lateral distance
e110 = 0;                                                   % first derivative of lateral distance
% e20 = X0(3)-X0_star(3);                                     % heading error
e220 = 0;                                                   % first derivative of heading error
e0 = [e10; e110; e20; e220];

%% controllers
% 1. longitudinal cruise control (PI)
Kp1 = 0.3;
Ki1 = 0.48;

% 2. lateral lane following control (MPC)
Q = 1.0 * eye(4);
R = 0.002;
h = 5;
tau1 = pi*0.1/6;
tau2 = pi/6;

[n_states,n_input] = size(B1_lat_d);

[M,F1,H_bar,H1,H2,b_bar,b1,b2] = MPC_vercterization(A_lat_d,B1_lat_d,Q,R,h,tau1,tau2);
[~,F2,~,~,~,~,~,~] = MPC_vercterization(A_lat_d2,B1_lat_d2,Q,R,h,tau1,tau2);

% constrained_MPC_pgdhb(e0,zeros(h,1),M,F1,H1,H2,b1,b2,h,4,1);


% for feedforward term
b1 = Theta6*Vx + Theta5*Rw*omegam/rg;
b2 = Theta3*L*Vx/2 + Rw*L*Theta4*omegam/(2*rg);

% 3. slow down when turning (adpaptive P)
% gain = min(1,max(0.5,cos(steering)));
% desired_longitudinal_speed = gain * desired_longitudinal_speed;
