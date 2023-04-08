function [constr, theta] = theta_goal_single(z, z_lb, z_ub)

    [bds_z1, theta_z1] = theta_p(z(:,1), 1, z_lb);
    [bds_z2, theta_z2] = theta_p(z(:,1), -1, -z_ub);
    constr = [bds_z1; bds_z2];
    
    N = size(z, 1);
    theta = sdpvar(N, 1);
    for t=1:N
        [c, theta(t)] = quant_and([theta_z1(t); theta_z2(t)]);
        constr = [constr; c];
    end
end