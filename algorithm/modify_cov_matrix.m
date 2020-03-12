function S = modify_cov_matrix( S, f )
% S ... covariance matrix calculated from a vector with first two elements
%       equal to (x,y) coordinates
% f ... desired angle of the covariance matrix part thta corresponds to (x,y)

R = [cos(-f), -sin(-f); sin(-f), cos(-f)] ;
Ss = S(1:2,1:2) ;

Ss = R'*((R*Ss*R').*eye(2))*R ;
S(1:2,1:2) = Ss ;