function [theta1, theta2] = IK(l1, l2, xh, zh, xf, zf)
l_square = (xh - xf)^2 + (zh - zf)^2;
A = -(l1^2 + l2^2 - l_square) / (2 * l1 * l2);
theta2 = acos(A);
B = l2 * sin(theta2) / (l1 + l2 * cos(theta2));
C = (xf - xh) / (zh - zf);
theta1 = atan(C) - atan(B);
end