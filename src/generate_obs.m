function points = generate_obs(center,radius, N)
    theta = linspace(0, 2*pi, N+1); theta(end) = [];
    x = center(1) + radius * cos(theta);
    y = center(2) + radius * sin(theta);
    points = [x', y'];
end

