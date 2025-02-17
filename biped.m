function xdot = biped(t, x, data)
    % Extract q and qdot from state vector x
    q = x(1:5);       % Joint angles
    qdot = x(6:10);   % Joint velocities

    % Extract parameters from data structure
    B = data.B;
    H = data.H;
    qref = data.qref;
    Kp = data.Kp;
    Kd = data.Kd;
    
    % Compute system dynamics using the precomputed function
    [D, ~, px, E, G, C] = biped_dynamics(q, qdot);

    % Compute feedback control torque τ using equation (2)
    h = H * q - qref;
    dhq = H;
    
    HD_inv_B = (dhq * (D \ B));  % (H D⁻¹ B)
    HD_inv_C = (dhq * (D \ (C * qdot + G)));  % H D⁻¹ (C qdot + ∇q P)
    
    tau = HD_inv_B \ (HD_inv_C - Kp * sin(h) - Kd * dhq * qdot);

    % Compute qddot using the system dynamics equation
    qddot = D \ (-C * qdot - G + B * tau);
    
    % Set xdot = [qdot; qddot]
    xdot = [qdot; qddot];
end
