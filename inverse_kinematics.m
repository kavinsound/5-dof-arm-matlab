function [a1, a2, a3, a4, a5] = inverse_kinem(x, y, z, r1, r2)
    base = 1.25;
    L1 = 9.1601;
    L2 = 8.0463;
    L3 = 4.7889;
    L4 = 2.0304;
    tip = 2.5245;
    pos = [x, y, z];
    
    r1 = deg2rad(r1);
    r2 = deg2rad(r2);

    tip_v = [sin(r2)*cos(r1), cos(r2), sin(r2)*sin(r1)]; %direction of end
    A = tip_v * tip; %vector of end

    B = pos + A; %target of rest of body
   
    L3 = L3 + L4;
    a1 = atan2(B(3), B(1));
    a1 = a1 + pi - 0.1098;
    l1_v = [0, L1];

    B_t = [sqrt(B(1)*B(1) + B(3)*B(3)), B(2)]; %convert to 2d problem
    %l1 + L2 * <sin(a2), cos(a2)> + L3 * <sin(a2+a3), cos(a2+a3)> = B_t
    
    X_prime = B_t(1);
    Y_prime = B_t(2) - l1_v(2);
    
    robot_system = @(p) [
    L2 * sin(p(1)) + L3 * cos(p(2)) - X_prime; ... % X residual
    L2 * cos(p(1)) + L3 * sin(p(2)) - Y_prime      % Y residual
    ];
    
    initial_guess = [pi/2, pi/4];
    options = optimoptions('fsolve', 'Display', 'off', 'Algorithm', 'levenberg-marquardt');

    [angles_sol, fval, exitflag] = fsolve(robot_system, initial_guess, options);
    
    

    theta1 = angles_sol(1);
    theta2 = angles_sol(2) - pi/2 + theta1;
    theta3 = angles_sol(2);
    
    theta1 = wrapToPi(theta1);
    theta2 = wrapToPi(theta2);
    theta3 = wrapToPi(theta3);

    alpha = atan2(B(3),B(1));
    u = [cos(theta3) * cos(alpha), sin(theta3), cos(theta3) * sin(alpha)];
    
    v_target = cross(u, tip_v);
    
    j = [0, 1, 0];

    % 3. Project the global up-axis onto the link's perpendicular plane
    v_ref = j - dot(j, u) * u;
    
    % 4. Normalize the reference vector
    v_norm = norm(v_ref);
    
    if v_norm > 1e-6
        v_ref = v_ref / v_norm; % This is your perfect "pointing up" reference
    end
    
    v_ref_proj = v_ref - dot(v_ref, u) * u;
    v_target_proj = v_target - dot(v_target, u) * u;
    
    % 3. Normalize the projected vectors
    v_ref_proj = v_ref_proj / norm(v_ref_proj);
    v_target_proj = v_target_proj / norm(v_target_proj);
    
    % 4. Determine cosine via dot product
    c = dot(v_ref_proj, v_target_proj);
    
    % 5. Determine sine via cross product scaled along axis u
    w = cross(v_ref_proj, v_target_proj);
    s = dot(w, u);
    
    % 6. Calculate the final signed angle in radians (-pi to pi)
    twist_angle = atan2(s, c);
    
    a4 = pi + twist_angle;

    a2 = theta1;
    a3 = theta2;
    
    ang = acos(dot(u, tip_v));

    a5 = pi/2 - ang;
end