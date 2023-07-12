function U = constrained_fmincon()






function [c,ceq] = sys_dynamics(x,y)
%% x = [h,u]
c = [];
V = V0;
for iter = 1:N
    V = MicroNole_dynamics(V,omega,steering,Ts,Theta,L, Rw, rg);
    X = MicroNole_kinematics(X,V(2),steering,Ts,L);
end
ceq = ;



