function [value, isterminal, direction] = ground_impact(t, x, data)
    % ground_impact: Detects when the swing foot hits the ground.
    % Inputs:
    %   t - Current time (unused in this case).
    %   x - Current state of the robot [q; qdot].
    %   data - Structure containing dynamics and parameters.
    % Outputs:
    %   value - Event condition (height of the swing foot).
    %   isterminal - If 1, stop the integration when the event occurs.
    %   direction - The direction of zero-crossing to trigger the event.

    % Extract q (joint angles) from state vector x
    q = x(1:5);
    qdot = x(6:10);
    % Compute the swing foot position using biped_dynamics
    %[~, ~, px, ~, ~, ~] = biped_dynamics(q, zeros(5, 1));
    [~, ~, px, ~, ~, ~] = biped_dynamics(q, qdot);
    
    % Swing foot height is the second component of px (y-coordinate)
    swing_foot_height = px(2);

    % Event condition: when swing foot height crosses zero
    value = swing_foot_height;  % The event triggers when this becomes 0
    isterminal = 1;             % Stop the integration
    direction = -1;             % Detect only when height decreases (positive to zero)

    fprintf('Time: %.2f, Swing foot height: %.3f, Joint angles (q): [%.2f %.2f %.2f %.2f %.2f]\n', ...
    t, swing_foot_height, q(1), q(2), q(3), q(4), q(5));

end