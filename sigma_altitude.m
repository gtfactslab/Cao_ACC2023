function cost = f(x, u, e, data, obs_y, obs_z, blocking_interval)
     
    state_centers = (x(1:blocking_interval:end, 1:6) + x(1:blocking_interval:end, 7:end))/2;
    
    [~, cov_f_star_y] = fit_params(obs_y(:, 1:end-1), obs_y(:, end), state_centers(:, 3));
    [~, cov_f_star_z] = fit_params(obs_z(:, 1:end-1), obs_z(:, end), state_centers(:, 3));
    
    std_f_star_y = sqrt(diag(cov_f_star_y));
    std_f_star_z = sqrt(diag(cov_f_star_z));
    
    cost = sum(std_f_star_y) + sum(std_f_star_z) - sum(state_centers(:, 3));

end