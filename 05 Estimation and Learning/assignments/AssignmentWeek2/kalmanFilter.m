function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    P = eye(4);
    R = eye(2);

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
%     % As an example, here is a Naive estimate without a Kalman filter
%     % You should replace this code
%     vx = (x - state(1)) / (t - previous_t);
%     vy = (y - state(2)) / (t - previous_t);
%     % Predict 330ms into the future
%     predictx = x + vx * 0.330;
%     predicty = y + vy * 0.330;
%     % State is a four dimensional element
%     state = [x, y, vx, vy];
    
    % Measurement
    z = [x; y];
    
    % Define the state transfer function and measurement function
    dt = t - previous_t;
    A = [1, 0, dt, 0; 0, 1, 0, dt; 0, 0, 1, 0; 0, 0, 0, 1];
    C = [1, 0, 0, 0; 0, 1, 0, 0];
    
    sig_m = 0.1 * eye(4);
    sig_o = 0.01 * eye(2);
    
    P = A * param.P * A' + sig_m;
    R = C * P * C' + sig_o;
    
    % Kalman Gain
    K = P * C' * inv(R + C * P * C');
    
    % Prediction
    state = A * state' + K * (z - C * A * state');
    predictx = state(1) + state(3) * 0.33;
    predicty = state(2) + state(4) * 0.33;
    state = state';
    
    param.P = P - K * C * P;
    
end
