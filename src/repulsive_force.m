function F_rep = repulsive_force(x, obstacle_points, rho_0, Krep)
% obstacle_points: Nx2 matrix whit contour points of the obstacle

F_rep = [0, 0];
for i = 1:length(obstacle_points)
    deltas = obstacle_points{i} - x;
    dists = vecnorm(deltas, 2, 2);
    [rho, idx] = min(dists);
    if rho <= rho_0 && rho > 0
        b = obstacle_points{i}(idx,:);
        grad_rho = (x - b) / norm(x - b);
        F_rep = F_rep + Krep *(1/rho - 1/rho_0) * (1/rho)^2 * grad_rho;
    end
end
F_rep = - F_rep;
end

