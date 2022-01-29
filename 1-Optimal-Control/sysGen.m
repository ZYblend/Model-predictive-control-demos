function [A,B]= sysGen(m,n)
% function [A,C]= sysGen(m,n)
% Inputs: 
%          m: [scalar] number of inputs
%          n: [scalar] number of states
% Outputs:
%          system: x^{+} = Ax + Bu
%
% @Written by Yu Zheng, Tallahassee, Florida, Aug. 2020

% generate properly normalized Gaussian matrix A with i.i.d. entries
A_gen = (1/sqrt(n)) * randn(n,n);
A = zeros(size(A_gen));
for col = 1:n
    A(:,col) = A_gen(:,col)/norm( A_gen(:,col) );
end
B_gen = (1/sqrt(n)) * randn(m,n);
B = zeros(size(B_gen));
for col = 1:n
    B(:,col) = B_gen(:,col)/norm( B_gen(:,col) );
end


% examinate observability
unob = rank(ctrb(A,B))-length(A); 
if unob == 0
    fprintf('fully controllable\n');
else
    fprintf('no fully controllable\n');
end

    
end
    