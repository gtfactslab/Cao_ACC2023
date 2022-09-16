function cost = f(x, u, e, data, obs, goal, block_int)
    
    % information gain (make sure this matches the actual observations in
    % number of states!)
    state_centers = (x(1:block_int:end, 1:4) + x(1:block_int:end, 5:8))/2;
%     centers = [state_centers, u];
    
    if numel(size(obs)) == 2
        % empirical testing shows that fitting should be faster
        if numel(obs(1, 1:end-1)) == 1
            [~, cov_f_star] = fit_params(obs(:, 1:end-1), obs(:, end), state_centers(:, 1));
        elseif numel(obs(1, 1:end-1)) == 2
            [~, cov_f_star] = fit_params(obs(:, 1:end-1), obs(:, end), state_centers(:, 1:2));
        else
            error("obs has too many independent variables")
        end
        
        std_f_star = sqrt(diag(cov_f_star));
    
        cost = 5*(std_f_star);
    
    elseif numel(size(obs)) == 3
        max_obs = obs(:, :, 2);
        min_obs = obs(:, :, 1);
        
        vmax = griddata(max_obs(:, 1), max_obs(:, 2), max_obs(:, 3), state_centers(:, 1), state_centers(:, 2));
        vmin = griddata(min_obs(:, 1), min_obs(:, 2), min_obs(:, 3), state_centers(:, 1), state_centers(:, 2));
        
        cost = 5*(abs(vmax - vmin));
    else
        error("your obs are wack");
    end

    
    
    % distance to goal
    norms = (vecnorm(state_centers(:, 1:2) - goal(1:2), 2, 2)).*exp(-cost);
    cost = sum(cost-norms); % we subtract because we want higher distance from goal to be punished

end