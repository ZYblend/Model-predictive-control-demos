function X_next = MicroNole_kinematics(X,Vx,steering,Ts,L)

Phi = X(3);

beta = atan(tan(steering)/2);
v = Vx*cos(beta);

x_dot = v * cos(Phi + beta);
y_dot = v * sin(Phi + beta);
Phi_dot = v * cos(beta) * tan(steering) / L;

X_dot = [x_dot;
         y_dot;
         Phi_dot];

X_next = X+X_dot*Ts;