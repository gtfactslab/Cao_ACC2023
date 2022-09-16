function x_dot = f(x_in, u_in, obs)  

    xhat = x_in(5:8);
    x = x_in(1:4);
    
    % occasionally run into issues with very close x/xhat
    if ~(all(x <= xhat) || all(xhat <= x))
        x = [min(x_in(1), x_in(5));
            min(x_in(2), x_in(6));
            min(x_in(3), x_in(7));
            min(x_in(4), x_in(8))];
        xhat = [max(x_in(1), x_in(5));
            max(x_in(2), x_in(6));
            max(x_in(3), x_in(7));
            max(x_in(4), x_in(8))];
    end

    input_uncertainty = 0.0;
    uhat = u_in + input_uncertainty;
    u = u_in - input_uncertainty;
   
%     a = u(1);
%     ahat = uhat(1);

%     tic
% empirical testing shows that interp should be slightly faster but the
% difference is minimal
    if numel(size(obs)) == 2
        % get params for the GP    
        if numel(obs(1, 1:end-1)) == 1
            [wmin, wmax] = disturbance_bounds_fitparam(x(1), xhat(1), obs); %1d
        elseif numel(obs(1, 1:end-1)) == 2
            [wmin, wmax] = disturbance_bounds_fitparam(x(1:2), xhat(1:2), obs); %2d
        else
            error("obs has too many independent variables")
        end
    elseif numel(size(obs)) == 3
        [wmin, wmax] = disturbance_bounds_interp(x(1:2), xhat(1:2), obs); %2d
    else
        error('your obs are wack');
    end
%     toc
    
%     wmin(wmin < 0) = 0;
%     wmin(wmin > 0.8) = 0.8;
%     wmax(wmax < 0) = 0;
%     wmax(wmax > 0.8) = 0.8;

% test that it's working with correct bounds, so we can eliminate
% possibility of bounds being incorrect when debugging
    x_dot = [d(x, u, wmin, xhat, uhat, wmax); d(xhat, uhat, wmax, x, u, wmin)];
    
    
end



function out = d_w1w2(w, what)
    % assuming 2d input
    values = [w(1)*w(2),
              w(1)*what(2),
              what(1)*what(2),
              what(1)*w(2)];
    if all(w <= what)
        out = min(values);
    elseif all(what <= w)
        out = max(values);
    else
        error("w (%s) and what (%s) are not component-wise comparable", mat2str(w), mat2str(what))
    end
end

function out = d(x, u, w, xhat, uhat, what)
    lr = 5.5 * 0.6;
    lf = 5.5 * 0.4;
    % x = [X, Y, psi, v]
    % u = [a, delta_f]
    
    beta_delta = beta(u(2));
    beta_delta_hat = beta(uhat(2));
    
    psi = x(3);
    psi_hat = xhat(3);
    
    mode = 1;
    if x(1) > xhat(1)
        mode = 2;
    end
    
    if ~(all(x <= xhat) || all(xhat <= x))
        error("x (%s) and xhat (%s) are not component-wise comparable", mat2str(x), mat2str(xhat))
    end
    
    dX = d_w1w2([x(4), d_cos(psi + beta_delta, psi_hat + beta_delta_hat)], [xhat(4), d_cos(psi_hat + beta_delta_hat, psi + beta_delta)]);
    dY = d_w1w2([x(4), d_sin(psi + beta_delta, psi_hat + beta_delta_hat)], [xhat(4), d_sin(psi_hat + beta_delta_hat, psi + beta_delta)]);
    dpsi = (1/lr)*d_w1w2([x(4), d_sin(beta_delta, beta_delta_hat)], [xhat(4), d_sin(beta_delta_hat, beta_delta)]);

    if isempty(w) || isempty(what)
        error('w or what empty');
    end
    dv = d_w1w2([(1 - what), u(1)], [(1 - w), uhat(1)]);
    
    out = [dX; dY; dpsi; dv];
    
    function out = beta(delta_f)
        out = atan(lr*tan(delta_f)/(lr+lf));
    end
end

function [wmin, wmax] = disturbance_bounds_fitparam(x, xhat, obs)
    % obs is assumed to be a strict list of the observations
    % of the GP
%     sprintf('emb_fitparam')
    x_lower = min(x, xhat);
    x_upper = max(x, xhat);
    spacing = (x_upper - x_lower)./10;

    if numel(x) == 1 %1D case
        x_space = [x_lower, x_lower:spacing:x_upper, x_upper]';
    elseif numel(x) == 2 %2D case
        [x1_vec, x2_vec] = meshgrid([x_lower(1), x_lower(1):spacing(1):x_upper(1), x_upper(1)], [x_lower(2), x_lower(2):spacing(2):x_upper(2), x_upper(2)]);
        x_space = [reshape(x1_vec, [], 1), reshape(x2_vec, [], 1)];
    else
        error("disturbance can only take 1d or 2d input")
    end
    
    sigma_confidence = 3;
    [fbs, cfs] = fit_params(obs(:, 1:end-1), obs(:, end), x_space);
    sfs = sqrt(diag(cfs));
    
    
    wmin = min(fbs - (sigma_confidence * sfs));
    wmax = max(fbs + (sigma_confidence * sfs));
    
    if isempty(wmin) || isempty(wmax)
        error('wmin or wmax empty');
    end
end

function [wmin, wmax] = disturbance_bounds_interp(x, xhat, obs)
    % obs is assumed to be a 3D vector containing the grid points of the
    % upper and lower bounds to be interpolated
%     sprintf('emb_interp')
    x_lower = min(x, xhat);
    x_upper = max(x, xhat);
    spacing = (x_upper - x_lower)./10;

    if numel(x) == 1 %1D case
        x_space = [x_lower, x_lower:spacing:x_upper, x_upper]';
        error("1D interpolation not yet supported")
    elseif numel(x) == 2 %2D case
        x1v = [x_lower(1), x_lower(1):spacing(1):x_upper(1), x_upper(1)];
        x2v = [x_lower(2), x_lower(2):spacing(2):x_upper(2), x_upper(2)];
        
        if isempty(x1v)
            x1v = x_lower(1);
        end
        if isempty(x2v)
            x2v = x_lower(2);
        end
        
        [x1_vec, x2_vec] = meshgrid(x1v, x2v);
        x_space = [reshape(x1_vec, [], 1), reshape(x2_vec, [], 1)];
    else
        error("disturbance can only take 1d or 2d input")
    end
    
    max_obs = obs(:, :, 2);
    min_obs = obs(:, :, 1);
    
    
    vmax = griddata(max_obs(:, 1), max_obs(:, 2), max_obs(:, 3), x_space(:, 1), x_space(:, 2));
    vmin = griddata(min_obs(:, 1), min_obs(:, 2), min_obs(:, 3), x_space(:, 1), x_space(:, 2));
    
    wmin = min(vmin);
    wmax = max(vmax);
    
    if isempty(wmin) || isempty(wmax)
        error('wmin or wmax empty');
    end
end

