function x = proj_onto_Halfspace(a,y,t)
%% function x = proj_onto_Halfspace(a,y,t)
% project vector y onto teh halfspace a.'*x <=t
% P(y) = y - (a.'*y - t)*a/||a||_2^2  if a.'*y > t
% P(y) = y                            otherwise
% Input:
%       
%
% Yu Zheng, Raslab, Florida State University, 2022
%


R = (a.' * y) - t;

if R > 0
    x = y - R * a/ norm(a)^2;
else
    x = y;
end