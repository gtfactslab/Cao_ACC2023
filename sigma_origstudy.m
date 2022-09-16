function cost = f(x, u, e, data, obs)
     
    state_centers = (x(:, 1:2) + x(:, 3:4))/2;
    
    centers = [state_centers, u];
    [~, cov_f_star] = fit_params(obs(:, 1:end-1), obs(:, end), centers);
    
    std_f_star = sqrt(diag(cov_f_star));
    
    cost = sum(std_f_star);

end