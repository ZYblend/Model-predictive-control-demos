function [e1,e2,Phi_dot,flag] = crosstrack_error(x,y,p,omegaz)
%% crosstrack error calculation
% Calculate the minimal crosstrack error and the heading error
% Inputs: 
%        - x,y: location of vehicle
%        - p: polynominal trajectory parameters f(x) = p(1)*x.^2 + p(2)*x + p(3)
%        - omegaz: heading of vehicle
% Output:
%        - e1: crosstrack error
%        - e2: heading error
%        - Phi_dot: curvature of trajectory
%        - flag: 1: counter clockwise; -1: clockwise
%
% Yu Zheng, Raslab, Florida State University, 2022

%% solve minimize 1/2(x-x0)^2 + (y-y0)^2
a = p(1);
b = p(2);
c = p(3);
g = [2*a^2, 3*a*b, 1+2*a*(c-y)+b^2, b*(c-y)-x];
x_candidate = real(roots(g));

%% find the minimal crosstrack error
f = @(z) a*z.^2 + b*z + c;
dist = @(z) sqrt( (z-x)^2+(f(z)-y)^2 );

num_candidate = length(x_candidate);
d_candidate = zeros(num_candidate,1);
for iter = 1:num_candidate
    d_candidate(iter) = dist(x_candidate(iter));
end
[d,I] = min(d_candidate);

x_star = x_candidate(I);
y_star = f(x_star);

%% identify the trajectory is counter clockwise or clockwise
% clockwise: x,y increase or decrease at same time
% counter clockwise: x increase (decrease) when y decrease (increase)
f_dot = @(z) 2*a*z + b;
flag = 1; % counter clockwise
if f_dot(x_star) < 0 
    % clockwise
    flag = -1;
end
    

%% calculate errors
e1 = flag * d * sign(x-x_star);     % lateral distance, at right, positive, at left, negative
e2 = omegaz - atan(f_dot(x_star));   % heading error

% 
Phi_dot = 2*a/(1+f_dot(x_star)^2);

end