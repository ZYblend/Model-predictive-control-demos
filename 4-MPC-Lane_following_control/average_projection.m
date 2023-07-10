function x = average_projection(ProjFun,y,iter_tol, error_tol)
%% function x = average_projection(ProjFun,y,iter_tol, erorr_tol)
% x_{k+1} = 1/m * sum(projFun(y))
%
% convergency proof: https://arxiv.org/pdf/0709.0109.pdf
%
% Input:
%       - ProjFun: cell array of all projecton functions
%       - y:  initial vector
%       -
% Output:
%       - x
%
iter = 0;
numSets = 5;
numElements = size(y,1);

while iter <= iter_tol
    iter = iter+1;
    x_proj = zeros(numElements,numSets);
    for i = 1:numSets
        x_proj(:,i) = ProjFun{i}(y);
    end
        
    x = sum(x_proj,2)/numSets;
%     disp(x)
    
    if norm(x-y) <= error_tol
        disp('average projection converged')
        break;
    end
    
    y = x;
end
    
    
        
