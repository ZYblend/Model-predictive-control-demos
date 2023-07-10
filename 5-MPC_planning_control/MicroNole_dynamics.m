function V_next = MicroNole_dynamics(V,omega,steering,Ts,Theta,L, Rw, rg)
% without motor dynamics


%% upacking inputs
yaw_rate = V(1);
Vx = V(2);
Vy = V(3);
delta = steering;
% Theta1 = Theta{1,1};
% Theta2 = Theta{1,2};
Theta3 = Theta{1,3};
Theta4 = Theta{1,4};
Theta5 = Theta{1,5};

%% dynamical model
Theta6 = Theta3*Theta5/Theta4;

% omega_dot = -Theta1 * omega + Theta2 * alpha;

A1 = (L/2) * Theta3 * delta * cos(delta);
B1 = (L/2) * Theta3 *(1-cos(delta));
C1 = ((1/2) * Theta3 * (1+cos(delta)) + Theta4) * (1/2) * L^2;
D1 = (Rw*L/(2*rg)) * Theta4 * sin(delta);
yaw_rate_dot = -C1*yaw_rate + A1*Vx + B1*Vy + D1*omega;

A2 = 2 * Theta5 + Theta6 * delta * sin(delta);
B2 = Theta6 * sin(delta);
C2 = L*Theta6*sin(delta)/2;
D2 = Theta5*Rw*(1+cos(delta))/rg;
Vx_dot = -A2*Vx + B2*Vy + C2*yaw_rate + yaw_rate*Vy + D2*omega;

A3 = Theta6 * delta * cos(delta);
B3 = Theta6 * (1+cos(delta));
C3 = L*(Theta6*(1-cos(delta))-2*Theta5)/2;
D3 = Theta5 * Rw *sin(delta)/rg;
Vy_dot = -B3*Vy + A3*Vx + C3*yaw_rate - yaw_rate*Vx + D3*omega;


V_dot = [yaw_rate_dot;
         Vx_dot;
         Vy_dot];

V_next = V+V_dot*Ts;

