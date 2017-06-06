function vec = convertMatrixToVec( mat )
%CONVERTMATRIXTOCOL Summary of this function goes here
%   Detailed explanation goes here

vec(1:4,1) = mat(1:4,1);
vec(5:8,1) = mat(1:4,2);
vec(9:12,1) = mat(1:4,3);
vec(13:16,1) = mat(1:4,4);
end

