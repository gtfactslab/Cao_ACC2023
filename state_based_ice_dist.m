function out = f(x)
    % assumes x is a horizonal concatenation of column vectors
    x_dims = size(x);
    x_dims = x_dims(1);

    % 1D mode
    if x_dims == 1
        obs = [-2, .1;
            -1, 0;
            0, .1;
            2, .2;
            3, .1;
            5, -.01;
            7, .1];
        [out, ~] = fit_params(obs(:, 1), obs(:, 2), x');
    % 2D mode
    elseif x_dims == 2
            obs = [-2, 0, -.01;
            0, 0, -.01;
            1, 0, .1;
            2, 1, .2;
            5, 0, 0;
            5, 5, 0.2;
            5, -5, -.01;
            7, 0, .1];
        [out, ~] = fit_params(obs(:, 1:2), obs(:, end), x');
    else
        error("Disturbance can only acccept 1-D or 2-D input");
    end

    
    % constrain coefficient to be in [0.2, 1]
    max_out = 0.8;
    min_out = 0;
    out(out > max_out) = max_out;
    out(out < min_out) = min_out;
    
end