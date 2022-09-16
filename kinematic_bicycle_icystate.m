function xdot = f(t, x, u)
    lr = 5.5 * 0.6;
    lf = 5.5 * 0.4;
    % x = [X, Y, psi, v]
    % u = [a, delta_f]
    psi = x(3);
    v = x(4);
    a = u(1);
    delta_f = u(2);
    
    xdot = [ v * cos(psi + beta(delta_f));
              v * sin(psi + beta(delta_f));
              (v/lr)*sin(beta(delta_f));
              a * (1-state_based_ice_dist(x(1)))]; % x(1) for 1d, [x(1); x(2)] for 2d
          
    function out = beta(delta_f)
        out = atan(lr*tan(delta_f)/(lr+lf));
    end

end