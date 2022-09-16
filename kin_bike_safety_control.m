function u = f(x, u_max, safe_pt, Ts)
    % temporary safety controller
    v = x(4);
    K = 1;
    u(1) = -K * (v/Ts);
    if abs(u(1)) > u_max
        u(1) = sign(u(1))*u_max;
    end
    
%     psi = x(3);
%     a = x(1:2);
%     b = safe_pt(1:2);
%     safe_vec = sign(v) * (b - a);
%     lr = 5.5 * 0.6;
%     lf = 5.5 * 0.4;
    
%     phi_dot = -psi/Ts;
%     u(2) = atan(tan(asin((lr/v)*phi_dot))*(lr+lf)/lr);
    u(2) = 0;
end