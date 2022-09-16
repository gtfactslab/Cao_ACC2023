function cost = f(x, u, e, data, obs, block_int)
    
    % information gain (make sure this matches the actual observations in
    % number of states!)
    state_centers = (x(1:block_int:end, 1:4) + x(1:block_int:end, 5:8))/2;
%     centers = [state_centers, u];
    
    if numel(obs(1, 1:end-1)) == 1
        [~, cov_f_star] = fit_params(obs(:, 1:end-1), obs(:, end), state_centers(:, 1));
    elseif numel(obs(1, 1:end-1)) == 2
        [~, cov_f_star] = fit_params(obs(:, 1:end-1), obs(:, end), state_centers(:, 1:2));
    else
        error("obs has too many independent variables")
    end
    
    std_f_star = sqrt(diag(cov_f_star));
    
    cost = sum(std_f_star);
end