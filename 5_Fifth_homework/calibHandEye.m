% Hand-eye calibration
%
% cam2target.csv format: 1 index, 2-4 rvec, 5-7 tvec.
% cam2target, transfer a point in camera frame to target(chessboard) frame.
% gripper2base, transfer a point in gripper frame to base frame.
% rvec: the axis-angle notation. rvec's direction is rotation direction, norm is the rotation angle (in rad)
% tvec: unit: mm
%
% Dong Yan  2022.01.4
%
% Modified by Umberto Castellani June 2024
% at the end, modified by Luca Ponti August 2024

clc; clear;
close all;

%% load data
fid = fopen('cam2target.csv', 'r');
A = textscan(fid, '%f,%f,%f,%f,%f,%f,%f');
rvec_t_c = [A{2}, A{3}, A{4}];
tvec_t_c = [A{5}, A{6}, A{7}];

fid = fopen('gripper2base.csv', 'r');
A = textscan(fid, '%f,%f,%f,%f,%f,%f,%f');
rvec_b_g = [A{2}, A{3}, A{4}];
tvec_b_g = [A{5}, A{6}, A{7}];

%% calculate relative transformation
N = 18;
T_b_g_list = zeros(4, 4, N);
T_t_c_list = zeros(4, 4, N);
% From target to camera
T_c_t_list = zeros(4, 4, N);

for i = 1:N
    %     fprintf("%d\n", i);
    r_t_c = rvec_t_c(i, :)';
    t_t_c = tvec_t_c(i, :)';
    R_t_c = Rodrigues(r_t_c);
    T_t_c = [R_t_c, t_t_c; 0, 0, 0, 1];

    r_b_g = rvec_b_g(i, :)';
    t_b_g = tvec_b_g(i, :)';
    R_b_g = Rodrigues(r_b_g);
    T_b_g = [R_b_g, t_b_g; 0, 0, 0, 1];

    %
    T_t_c_list(:, :, i) = T_t_c;
    T_b_g_list(:, :, i) = T_b_g;

    %Umbe: transformation from calibration object to camera (as in Tsai)
    T_c_t_list(:, :, i) = inv(T_t_c);
end

%This is the Tsai method as described in the original paper:
[cam2grip, err1] = TSAIleastSquareCalibration(T_b_g_list, T_c_t_list);
% Ground-truth (measured by a ruler)
% R: Identity matrix (I)
% t: [-57, 65, 20] mm

%% calculate Gij and Cij
Cij_list = [];
Gij_list = [];

for k = 1:N - 1
    i = k;
    j = k + 1;
    Cij = inv(T_t_c_list(:, :, i)) * T_t_c_list(:, :, j);
    Gij = inv(T_b_g_list(:, :, i)) * T_b_g_list(:, :, j);
    Cij_list = [Cij_list, Cij];
    Gij_list = [Gij_list, Gij];
end

vison = 1;

if (vison)
    %Plot cameras
    figure(1);
    plotRef(eye(3), [0, 0, 0]', 'Object');
    grid on;
    axis equal;
    hold on;

    for i = 1:N
        R = T_t_c_list(1:3, 1:3, i);
        T = T_t_c_list(1:3, 4, i);
        plotRef(R, T, num2str(i));
    end

    %Plot grippers
    figure(2);
    plotRef(eye(3), [0, 0, 0]', 'Base');
    grid on;
    axis equal;
    hold on;

    for i = 1:N
        R = T_b_g_list(1:3, 1:3, i);
        T = T_b_g_list(1:3, 4, i);
        plotRef(R, T, num2str(i));
    end

end

% X is T_g_c when given GX = XC
% This is another version of Tsai, that use Rodrigues to estimate the
% rotation matrix after the computation of rotation angle
%
newTsai = 1;

if (newTsai)
    T_g_c = tsai(Gij_list, Cij_list)
end

% Ground-truth (measured by a ruler)
% R: Identity matrix (I)
% t: [-57, 65, 20] mm

% This version use the Tsai version with Rodrigues
vison = 1;

if (vison)
    %Plot cameras
    figure(2);
    grid on;
    axis equal;
    hold on;
    M_out_store = eye(4);

    for i = 1:N
        %Camera to robot
        C_out = T_b_g_list(:, :, i) * T_g_c;
        plotRef(C_out(1:3, 1:3), C_out(1:3, 4), num2str(i));
        % Calibration object to robot
        R_c = (T_t_c_list(1:3, 1:3, i))';
        T_c = -R_c * T_t_c_list(1:3, 4, i);
        M_c = [R_c T_c; 0 0 0 1];
        M_out = T_b_g_list(:, :, i) * T_g_c * M_c;
        %plotRef(M_out(1:3,1:3), M_out(1:3,4), num2str(i));
        plotRef(M_out(1:3, 1:3), M_out(1:3, 4), 'Object');
        Dif = inv(M_out_store) * M_out;
        Dif
        M_out_store = M_out;
    end

end

%This version use the Tsai version like in the original paper:
vison = 1;

if (vison)
    %Plot cameras
    figure(3);
    %plotRef(eye(3), [0,0,0]', 'Base');
    %grid on;
    %axis equal;
    %hold on;
    M_out_store = eye(4);

    for i = 1:N
        %
        % Robot reference system
        plotRef(eye(3), [0, 0, 0]', 'Base');
        grid on;
        axis equal;
        hold on;
        %Camera to robot
        C_out = T_b_g_list(:, :, i) * cam2grip;
        plotRef(C_out(1:3, 1:3), C_out(1:3, 4), num2str(i));
        % Calibration object to robot
        R_c = (T_t_c_list(1:3, 1:3, i))';
        T_c = -R_c * T_t_c_list(1:3, 4, i);
        M_c = [R_c T_c; 0 0 0 1];
        M_out = T_b_g_list(:, :, i) * cam2grip * M_c;
        %Plot calib object to robot
        plotRef(M_out(1:3, 1:3), M_out(1:3, 4), 'Object');
        Dif = inv(M_out_store) * M_out;
        Dif
        M_out_store = M_out;
        %Plot grip to robot
        R = T_b_g_list(1:3, 1:3, i);
        T = T_b_g_list(1:3, 4, i);
        plotRef(R, T, num2str(i));
        %hold off
    end

end


