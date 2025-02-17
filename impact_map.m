function Delta = impact_map(x, data)
    % IMPACT_MAP Computes the post-impact state of the biped robot
    % Input:
    %   x    - Pre-impact state [q; qdot]
    %   data - Structure array containing robot parameters and functions
    % Output:
    %   Delta - Post-impact state [q; qdot_new]

    % Extract pre-impact states
    q = x(1:5);       % Joint angles
    qdot = x(6:10);   % Joint velocities

    % Extend qdot to match Dbar dimensions (7x1)
    %qdot_full = [qdot; 0; 0]; % Assuming x1dot and x2dot are initially 0

    % Extract required functions from data
    [~, Dbar, ~, E, ~, ~] = biped_dynamics(q, qdot); % Call dynamics function

    % Debugging sizes
    disp('Dbar size:');
    disp(size(Dbar));
    disp('E size:');
    disp(size(E));
    

    % Compute Delta_qdot using the corrected formula
    % n = size(Dbar, 1); % Rows of Dbar
    % m = size(E, 1);    % Rows of E
    M = [Dbar, -E'; E, zeros(2,2)];
    RHS = [Dbar*[eye(5);zeros(2,5)];zeros(2,5)]; 
    Delta_qdot = [eye(5), zeros(5,4)] * (M \ RHS);
    q_new = q;
    qdot_new = Delta_qdot * qdot;

    % Relabeling (swap legs)
    %R = [1, 0, 0, 0, 0;
     %0, 0, 0, -1, 0;
     %0, 0, -1, 0, 0;
     %0, -1, 0, 0, 0;
     %0, 0, 0, 0, 1];
    R = [1,1,1,1,0;
        0,0,0,-1,0;
        0,0,-1,0,0;
        0,-1,0,0,0;
        0,0,-1,0,1
        
    ];

    %d = [pi/12; 2*pi; 2*pi; 2*pi-pi/12; 0];
    d = [-pi;0; 0; 0; -pi];
    


    % Flip the angles of the swing leg
    % d = zeros(5,1); % No offset
    q_new = R * q + d;
    qdot_new = R * qdot_new;

    % Return the post-impact state
    Delta = [q_new; qdot_new];
end
