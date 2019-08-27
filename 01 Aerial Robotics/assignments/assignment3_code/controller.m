function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thrust
F = 0;

% Moment
M = zeros(3,1);

k_d = 10;
k_p = 100;

e_p = des_state.pos - state.pos;
e_v = des_state.vel - state.vel;

r_des_ddot = des_state.acc + k_d * e_v + k_p * e_p;

F = params.mass * (params.gravity + r_des_ddot(3));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
k_d_attitude = 2;
k_p_attitude = 100;

psi_t = des_state.yaw;
phi_des = (r_des_ddot(1) * sin(psi_t) - r_des_ddot(2) * cos(psi_t)) / params.gravity;
theta_des = (r_des_ddot(1) * cos(psi_t) + r_des_ddot(2) * sin(psi_t)) / params.gravity;

e_rot = [phi_des; theta_des; des_state.yaw] - state.rot;
e_omega = [0; 0; des_state.yawdot] - state.omega;

M = k_p_attitude * e_rot + k_d_attitude * e_omega;
% =================== Your code ends here ===================

end
