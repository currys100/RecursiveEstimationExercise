function mat = convertVecToMatrix( vec )
%CONVERTCOLTOMATRIX Summary of this function goes here
%   Detailed explanation goes here

mat(1:4,1) = vec(1:4,1);
mat(1:4,2) = vec(5:8,1);
mat(1:4,3) = vec(9:12,1);
mat(1:4,4) = vec(13:16,1);
end

