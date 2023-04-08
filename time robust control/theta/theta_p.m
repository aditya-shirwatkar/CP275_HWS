function [constr, theta, z_p] = theta_p(x, A, b)
% Ax >= b

[bds, z_p] = qual_mu(x, A, b);
[c, theta] = theta_plus(z_p);

constr = [bds; c];
end