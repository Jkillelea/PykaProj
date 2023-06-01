clear all;
close all;
clc;

logfile = dlmread('log1.csv');
% logfile = dlmread('log2.csv');

t      = logfile(:, 1);
gps    = logfile(:, 2);
laser1 = logfile(:, 3);
laser2 = logfile(:, 4);

% State vector
x = [
    0;  % MSL
    0;  % dMSL
    0;  % AGL
    ];

% State update matrix
A = [
%   MSL dMSL AGL
    1,  1,   0; % MSL = MSL + dMSL
    0,  1,   0; % dMSL = dMSL
    0,  1,   1; % AGL = AGL + dMSL
    ];

% initial state estimation covariance
P = ones(length(x));

% state estimation uncertainty growth (per iteration)
Q = [
%   MSL     dMSL      AGL
    0.01,   1,        1; % MSL
    1,      0.01,    10; % dMSL
    1,      10,       0.01; % AGL
    ];

% State to measurement matrix
H = [
%   MSL dMSL AGL
    1,  0,   0;  % GPS     = MSL
    0,  1,   0;  % dGPS    = dMSL
    0,  0,   1;  % laser1  = AGL
    0,  0,   1;  % laser2  = MSL - Terrain
     ];

dgps  = [0; diff(gps)];
data = [gps, dgps, laser1, laser2];

R = [
    10.5038   10.5059    3.1679    3.1304
    10.5059   21.0118    3.1664    3.1409
    3.1679    3.1664     2.4990    1.1770
    3.1304    3.1409     1.1770   11.4385
    ];

state_estimation_result = zeros(length(t), length(x));

for i = 1:length(t)
    %% Prediction
    x_next = A * x;
    if isnan(x_next)
        fprintf('x_next is NaN (iteration %d)\n', i);
        break;
    end

    % Update uncertainty
    P_next = A * P * A' + Q;
    if any(isnan(P_next))
        fprintf('P_next is NaN (iteration %d)\n', i);
        disp(P_next)
        break;
    end

    %% Measure
    z = data(i, :)';
    if any(isnan(z))
        fprintf('z is NaN (iteration %d)\n', i);
        disp(z)
        break;
    end

    %% Kalman gain
    K = P_next * H' * (H * P_next * H' + R)^-1;
    if isnan(K)
        fprintf('K is NaN (iteration %d)\n', i);
        break;
    end

    %% Update
    x = x_next + K * (z - (H * x_next));
    if isnan(x) || abs(x) == Inf
        fprintf('x is NaN (iteration %d)\n', i);
        break;
    end

    P = P_next - K * H * P_next;
    if isnan(P)
        fprintf('P is NaN (iteration %d)\n', i);
        break;
    end

    state_estimation_result(i, :) = x;
end

figure; hold on; grid on;
plot(t, gps,    'DisplayName', 'gps');
plot(t, laser1, 'DisplayName', 'laser1');
plot(t, laser2, 'DisplayName', 'laser2');
% plot(t, state_estimation_result(:, 1)); % AGL
plot(t, state_estimation_result(:, 3)); % AGL
legend('show');

