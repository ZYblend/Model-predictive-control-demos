function vX = Orthogonal_proj_onto_multiConvexSet(cProjFun,vY,numIterations, stopThr)
% Input:
%   - cProjFun      -   Array of Projection Functions.
%                       Cell array of anonymouse functions which each is a
%                       projection into a sub space.
%                       Structure: Cell Array.
%                       Type: NA.
%                       Range: NA.
%   - vY            -   Input Vector.
%                       Input vector to be projected.
%                       Structure: Vector (m x 1).
%                       Type: 'Single' / 'Double'.
%                       Range: (-inf, inf).
%   - numIterations -   Number of Iterations.
%                       Sets the number of iterations of the algorithm.
%                       Structure: Scalar.
%                       Type: 'Single' / 'Double'.
%                       Range: {1, 2, ...}.
%   - stopThr       -   Stopping Threshold.
%                       Sets the threshold between consecutive iterations
%                       for stopping the algorithm.
%                       Structure: Scalar.
%                       Type: 'Single' / 'Double'.
%                       Range: (0, inf).
% References
%   1.  Quadratic Optimization of Fixed Points of Non Expensive Mappings in Hilbert Space (https://doi.org/10.1080/01630569808816822).
%
% Cite:
% https://github.com/RoyiAvital/StackExchangeCodes/blob/master/Mathematics/Q1492095/HybridOrthogonalProjectionOntoConvexSets.m
% 
%
numSets     = size(cProjFun, 1);
numElements = size(vY, 1);

vX = vY;
vU = vX;
vT = vX;

vW = rand(numSets, 1);
vW = vW / sum(vW);

for ii = 1:numIterations
    
    vU(:) = vX;
    
    % See Quadratic Optimization of Fixed Points of Non Expensive Mappings
    % in Hilbert Space (Page 18, Equation 44)
    vT(:) = 0;
    for jj = 1:numSets
        vT(:) = vT + (vW(jj) * cProjFun{jj}(vX));
    end
    paramLambdaN = 1 / (ii + 1);
    vX(:) = (paramLambdaN * vY) + (1 - paramLambdaN) * vT;
    
    stopCond = sum(abs(vU - vX)) < stopThr;
    
    if(stopCond)
        disp('found hybrid solution')
        break;
    end
end
end