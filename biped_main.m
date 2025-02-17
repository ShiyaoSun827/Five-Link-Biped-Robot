clearvars; close all; clc;
% 1. 定义机器人参数
syms l1 l2 l3 m1 m2 m3 m4 m5 m6 g real
l1 = 0.5; l2 = 0.5; l3 = 0.3; % 连杆长度
m1 = 0.05; m2 = 0.5; m3 = 0.3; m4 = 0.5; m5 = 0.05; m6 = 0.5;% 连杆质量
g = 9.81; % 重力加速度

% 2. 定义符号变量
syms t q1 q2 q3 q4 q5 x1 x2 q1dot q2dot q3dot q4dot q5dot x1dot x2dot tau5 tau2 tau3 tau4 real
q = [q1; q2; q3; q4; q5];         % 关节角度向量
qdot = [q1dot; q2dot; q3dot; q4dot; q5dot]; % 角速度变量向量
tau = [tau2; tau3; tau4;tau5];   % 力矩
%x1 站立脚在地面上的水平位移
%x2 站立脚在地面上的垂直位移
x = [x1; x2];   
qbar = [q; x];               % 未固定机器人的位置状态向量
qbardot = [qdot; x1dot; x2dot];   % 未固定机器人的速度状态向量


%3.a 定义质心位置公式：以机器人惯性坐标系为参考
% 站立腿脚底的位置是 [x1, x2]
% m1: 站立脚底
%r_m1 = x;
r1 = [qbar(6); qbar(7)];
% m2: 站立腿膝关节位置
%r_m2 = r_m1 + l1 * [cos(q1); sin(q1)];
r2 = r1 + l1 * [cos(qbar(1)); sin(qbar(1))];

% m3: 躯干的连接位置
%r_m3 = r_m2 + l2 * [cos(q1 + q2); sin(q1 + q2)];
r3 = r2 + l2 * [cos(qbar(1) + qbar(2)); sin(qbar(1) + qbar(2))];
% m4: 摆动腿膝关节位置
%r_m4 = r_m1 + l1 * [cos(q1); sin(q1)] + l2 * [cos(q1 + q4); sin(q1 + q4)];
%r4 = r1 + l1 * [cos(qbar(1)); sin(qbar(1))] + l2 * [cos(qbar(1) + qbar(4)); sin(qbar(1) + qbar(4))];
r4 = r3 + l2 * [cos(qbar(1)+qbar(2)+qbar(3)); sin(qbar(1)+qbar(2)+qbar(3))];
% m5: 摆动脚的位置
%r_m5 = r_m4 + l1 * [cos(q1 + q4 + q5); sin(q1 + q4 + q5)];
r5 = r4 + l1 * [cos(qbar(1)+qbar(2)+qbar(3)+qbar(4)); sin(qbar(1)+qbar(2)+qbar(3)+qbar(4))];
% m6: 躯干末端的位置
%r_m6 = r_m3 + l3 * [cos(q1 + q2 + q3); sin(q1 + q2 + q3)];
r6 = r3 + l3 * [cos(qbar(1) + qbar(2) + qbar(5)); sin(qbar(1) + qbar(2) + qbar(5))];




% 3.b. jacobian velocity
% r1 v
r1dot = jacobian(r1, qbar) * qbardot;

% r2 v
r2dot = jacobian(r2, qbar) * qbardot;

% r3 v
r3dot = jacobian(r3, qbar) * qbardot;

% r4 v
r4dot = jacobian(r4, qbar) * qbardot;

% r5 v
r5dot = jacobian(r5, qbar) * qbardot;

% r6 v
r6dot = jacobian(r6, qbar) * qbardot;



%Since they are massless rods
% Define the total kinetic energy of the robot
%K=1/2*m*(rc1dot'*rc1dot+rc2dot'*rc2dot)+1/2*Iz*(q1dot^2+(q1dot+q2dot)^2);
%K=simplify(K);

%3.c No need for the rotational part
% Ki = (1/2) * mi * rdot^T * rdot
K1 = (1/2) * m1 * (r1dot.' * r1dot);
K2 = (1/2) * m2 * (r2dot.' * r2dot);
K3 = (1/2) * m3 * (r3dot.' * r3dot);
K4 = (1/2) * m4 * (r4dot.' * r4dot);
K5 = (1/2) * m5 * (r5dot.' * r5dot);
K6 = (1/2) * m6 * (r6dot.' * r6dot);
K = K1 + K2 + K3 + K4+ K5 + K6;
%K=simplify(K);
K = vpa(K)
% 3d

% Extract the square symmetric matrix of the kinetic energy
%Dbar=simplify(hessian(K,qbardot));
Dbar=hessian(K,qbardot);
%3e
% Extract the matrix of the kinetic energy of the pinned robot
D = Dbar(1:5,1:5);


%4.Define the potential energy of the pinnedrobot
%P1 = -m1 * g * r1(2); 
%P2 = -m2 * g * r2(2); 
%P3 = -m3 * g * r3(2); 
%P4 = -m4 * g * r4(2); 
%P5 = -m5 * g * r5(2); 
%P6 = -m6 * g * r6(2); 
P1 = m1 * g * x2; % m1 potential
P2 = m2 * g * (l1 * sin(q1)); % m2 potential
P3 = m3 * g * (l1 * sin(q1) + l2 * sin(q1 + q2)); % m3 potential
P4 = m4 * g * (l1 * sin(q1) + l2 * sin(q1 + q4)); % m4 potential
P5 = m5 * g * (l1 * sin(q1) + l2 * sin(q1 + q4) + l1 * sin(q1 + q4 + q5)); % potential
P6 = m6 * g * (l1 * sin(q1) + l2 * sin(q1 + q2) + l3 * sin(q1 + q2 + q3)); % m6 potential

%P = simplify(P1 + P2 + P3 + P4 + P5 + P6);
%G = simplify(jacobian(P, q).');%transpose
P = vpa(P1 + P2 + P3 + P4 + P5 + P6);
G = vpa(jacobian(P, q).');%transpose

%5.Input matrix of pinned robot
B = [0 0 0 0;  % q1 in unactuated
     1 0 0 0;  % directly actuated
     0 1 0 0;  % directly actuated
     0 0 1 0;  % directly actuated
     0 0 0 1]; % directly actuated
%6.Computation of matrix C(q,qdot) of pinned robot
C = sym(zeros(5, 5)); % init 5x5
for k = 1:5
    for j = 1:5
        for i = 1:5
            
            C(k, j) = C(k, j) + (1/2) * (diff(D(k, j), q(i)) + diff(D(k, i), q(j)) - diff(D(i, j), q(k))) * qdot(i);
        end
    end
end

%7 Define symbolic variables px and E containing px(q) and the expression above for E(q).

px = r5 - r1;
dpx_q = jacobian(px, q);
I2 = eye(2);
%disp('Size of px:');
%disp(size(px));
%disp('Size of dpx_q:');
%disp(size(dpx_q));
%disp('Size of I2:');
%disp(size(I2));

E = [dpx_q, eye(2)];
%E = [dpx_q, zeros(2,2); zeros(2,5), eye(2)];
%disp(size(E));
%8 Turn the symbolic expressions in D,Dbar,px,E,C,G into Matlab functions with appropriate inputs.
%D = Dbar(1:5,1:5);
%%Dfun = matlabFunction(D, 'Vars', {q}, 'File', 'D_matrix');
%Dbar=simplify(hessian(K,qbardot));
%%Dbarfun = matlabFunction(Dbar, 'Vars', {q}, 'File', 'Dbar_matrix');
%px = r5 - r1;
%%pxfun = matlabFunction(px, 'Vars', {q}, 'File', 'px_vector');
%E = [dpx_q; I2];
%%Efun = matlabFunction(E, 'Vars', {q}, 'File', 'E_matrix');
%C = sym(zeros(5, 5)); % init 5x5
%%Cfun = matlabFunction(C, 'Vars', {[q; qdot]}, 'File', 'C_matrix');
%G = simplify(jacobian(P, q).');%transpose
%%Gfun = matlabFunction(G, 'Vars', {q}, 'File', 'G_vector');
matlabFunction(D, Dbar, px, E, G, C, ...
    'Vars', {q, qdot}, ...
    'File', 'biped_dynamics', ...
    'Outputs', {'D_out', 'Dbar_out', 'px_out', 'E_out', 'G_out', 'C_out'});

% 9. Create a structure array named data containing these objects: 
%%data.Dfun = Dfun;        % Dfun
%%data.Dbarfun = Dbarfun;  % Dbarfun

%%data.Efun = Efun;        % Efun
%%data.Cfun = Cfun;        % Cfun
%%data.Gfun = Gfun;        % Gfun
%%data.B = B;              %  B
data.biped_dynamics = @biped_dynamics; 
data.B = B;
%part 3 ODE function and control design
%1.define numerical variables

q2ref = pi / 6;
q3ref = pi + pi / 6;
q4ref = -pi / 6;
q5ref = -pi / 6;
qref = [q2ref; q3ref; q4ref; q5ref];  


H = [zeros(4,1),eye(4)];
h = H *q -qref;
dhq = H;

Kp = diag([400, 400, 400, 400]);  
Kd = diag([40, 40, 40, 40]);  

%HD_inv_B = simplify(dhq * (D \ B));
%HD_inv_C = simplify(dhq * (D \ (C * qdot + jacobian(P, q).')));
%tau = simplify((HD_inv_B) \ (HD_inv_C - Kp * sin(h) - Kd * dhq * qdot));

            
data.qref = qref;        
data.Kp = Kp;            
data.Kd = Kd;            
data.H = H;              
%data.pxfun = pxfun;
%data.tau = tau;
save('biped_main', 'data');

%2.Compute the quantity and verify that it is not zero.
%rank 1 left-annihilator of B
B_perp = [1 0 0 0 0];

syms theta real
sigma_theta = [theta; q2ref; q3ref; q4ref; q5ref];

sigma_theta_dot = diff(sigma_theta, theta);

D_sigma = subs(D, q, sigma_theta);

%B_perp_D_sigma_sigma_dot = simplify(B_perp * D_sigma * sigma_theta_dot);
B_perp_D_sigma_sigma_dot = vpa(B_perp * D_sigma * sigma_theta_dot);
is_constant = isAlways(diff(B_perp_D_sigma_sigma_dot, theta) == 0);

disp('B^⊥ D(σ(θ)) σ''(θ) :');
disp(B_perp_D_sigma_sigma_dot);
if is_constant
    disp('B^⊥ D(σ(θ)) σ''(θ) is a constant and independent of θ.');
else
    disp('B^⊥ D(σ(θ)) σ''(θ) is NOT a constant!');
end

if any(B_perp_D_sigma_sigma_dot(:) ~= 0)
    disp('Result is nonzero.');
else
    disp('Result is zero. ');
end
% 3.Create an ODE function biped.m accepting the structure array data.

% 5.Simulation

% 2.Using the command odeset, set up integrations options in a structure named ops
ops = odeset('AbsTol', 1e-8, 'RelTol', 1e-8, 'Events', @(t, x) ground_impact(t, x, data));

% 3.Define the initial condition
q0 = [pi/3; pi/4; pi - pi/6; -pi/3; -pi/6]; % Initial joint angles
qdot0 = zeros(5,1); % Initial joint velocities
x0 = [q0; qdot0]; % Full initial state

% 4.simulate the robot until first impact occurs 
[t1, x1, te1, xe1] = ode45(@(t, x) biped(t, x, data), 0:5e-2:10, x0, ops);

% 5.using the function impact_map(xe1’,data) compute a post-impact state
x_post = impact_map(xe1', data);

% 6.Integrate a second time until next impact, saving time and state samples in vectors t2,x2
[t2, x2, te2, xe2] = ode45(@(t, x) biped(t, x, data), te1:5e-2:10, x_post, ops);

%7.Using the provided code snippet as a reference, create an animation of your results

run("animation_snippet.m")