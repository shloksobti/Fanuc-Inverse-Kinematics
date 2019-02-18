% Lab 2
% dhtf.m

function tf = dhtf(alpha, a, d, theta)
%{
Input: 4 DH parameters
Output: 4-by-4 transformation matrix
%}
tf = [cos(theta)            -sin(theta)           0           a;
      sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d;
      sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha)  cos(alpha)*d;
      0                     0                     0           1];
end