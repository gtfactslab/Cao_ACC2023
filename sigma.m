function cost = f(x, u, e, data, obs)
     
    x_end = (x(:, 4) + x(:, 2))/2;
    [~, cov_f_star] = fit_params(obs(:, 1), obs(:, 2), x_end);
    
    std_f_star = sqrt(diag(cov_f_star));
    
    cost = sum(std_f_star);

end