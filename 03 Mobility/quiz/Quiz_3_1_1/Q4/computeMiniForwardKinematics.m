function [endeff] = computeMiniForwardKinematics(rads1,rads2)

endeff = [0,0];
Ls = 1;
Ll = 2;

x = Ls * cos(rads1);
y = Ls * sin(rads1);

beta = rads1 - rads2;
alpha = (2 * pi - beta) / 2 + rads1;

theta = alpha - rads1;
gamma = asin(Ls * sin(theta) / Ll);
omega = pi - theta - gamma;
l = Ll/sin(theta) * sin(omega);

endeff = [l * cos(alpha), l * sin(alpha)];