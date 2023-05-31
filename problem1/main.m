clear all;
close all;
clc;

% logfile = dlmread('log1.csv');
logfile = dlmread('log2.csv');

t      = logfile(:, 1);
gps    = logfile(:, 2);
laser1 = logfile(:, 3);
laser2 = logfile(:, 4);

% State vector
x = [0;  % AGL
     0;  % dAGL
     0;  % Terrain
     0;  % dTerrain
     ];
% State update matrix
A = [
%   AGL dAGL Terrain dTerrain
    1,  1,   0,     -1;
    0,  1,   0,      0;
    0,  0,   1,      1;
    0, -1,   0,      1;
    ];

% initial state estimation covariance
P = ones(length(x));

% state estimation uncertainty growth (per iteration)
Q = [0.001];

% State to measurement matrix
%   agl dagl Terrain dTerrain
H = [
    1,  0,  1,      0;  % GPS     = AGL + Terrain
    0,  1,  0,     -1;  % dGPS    = dAGL - dTerrain
    1,  0,  0,      0;  % laser1  = AGL
    0,  1,  0,     -1;  % dlaser1 = dAGL - dTerrain
    1,  0,  0,      0;  % laser2  = AGL
    0,  1,  0,     -1;  % dlaser2 = dAGL - dTerrain
     ];

dgps    = [0; diff(gps)];
dlaser1 = [0; diff(laser1)];
dlaser2 = [0; diff(laser2)];

data = [gps, dgps, laser1, dlaser1, laser2, dlaser2];

R = [
    5.6191e-01   3.7912e-04  -1.4529e-02   1.8362e-04  -8.5454e-03   1.4775e-04
    3.7912e-04   2.6099e-05  -7.2327e-05   2.4852e-06  -8.1877e-05   3.0357e-06
   -1.4529e-02  -7.2327e-05   2.7869e-02   1.2229e-04   2.7874e-02   1.5943e-04
    1.8362e-04   2.4852e-06   1.2229e-04   2.8554e-04  -1.3987e-04   5.9946e-05
   -8.5454e-03  -8.1877e-05   2.7874e-02  -1.3987e-04   2.9542e-02   1.2854e-04
    1.4775e-04   3.0357e-06   1.5943e-04   5.9946e-05   1.2854e-04   2.9498e-04
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
plot(t, state_estimation_result(:, 1)); % AGL
plot(t, state_estimation_result(:, 3)); % Terrain
legend('show');

