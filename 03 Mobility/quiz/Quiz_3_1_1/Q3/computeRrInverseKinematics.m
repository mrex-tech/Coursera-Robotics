function [rads1,rads2] = computeRrInverseKinematics(X,Y)

syms theta1 theta2 ;

L1 = 1;
L2 = 1;
L = sqrt(X^2 + Y^2);

if X ~= 0
    theta = atan(Y / X);
else
    theta = pi / 2;
end

alpha = acos((L^2 + L1^2 - L2^2) / (2 * L * L1));
beta = acos((L^2 + L2^2 - L1^2) / (2 * L * L2));

rads1 = theta - alpha;
rads2 = alpha + beta;
