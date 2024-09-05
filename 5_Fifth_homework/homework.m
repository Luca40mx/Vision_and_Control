clearvars;
close all;          
clc;          

%%% Importazione dei dati

fileID_cam_target = fopen('cam2target.csv', 'r');
data_cam_target = textscan(fileID_cam_target, '%f,%f,%f,%f,%f,%f,%f');
fclose(fileID_cam_target);

fileID_grip_base = fopen('gripper2base.csv', 'r');
data_grip_base = textscan(fileID_grip_base, '%f,%f,%f,%f,%f,%f,%f');
fclose(fileID_grip_base);

% Estrazione delle rotazioni e traslazioni dai dati
rotations_cam_target = [data_cam_target{2}, data_cam_target{3}, data_cam_target{4}];
translations_cam_target = [data_cam_target{5}, data_cam_target{6}, data_cam_target{7}];

rotations_grip_base = [data_grip_base{2}, data_grip_base{3}, data_grip_base{4}];
translations_grip_base = [data_grip_base{5}, data_grip_base{6}, data_grip_base{7}];

num_samples = 18; % TODO: da modificare! rendi generico!

%%% Creazione delle matrici di trasformazione

% Inizializzazione delle matrici 4x4 per ogni campione
transforms_grip_to_base = zeros(4, 4, num_samples); 
transforms_cam_to_target = zeros(4, 4, num_samples); 

% Costruzione delle matrici di rototraslazione per ogni campione
for i = 1:num_samples
    % Generazione delle matrici di rotazione usando Rodrigues
    rotation_matrix_grip_base = Rodrigues(rotations_grip_base(i,:)');
    translation_vector_grip_base = translations_grip_base(i, :)';
    transforms_grip_to_base(:,:,i) = [rotation_matrix_grip_base, translation_vector_grip_base; 0 0 0 1];

    rotation_matrix_cam_target = Rodrigues(rotations_cam_target(i,:)');
    translation_vector_cam_target = translations_cam_target(i, :)';
    transforms_cam_to_target(:,:,i) = [rotation_matrix_cam_target, translation_vector_cam_target; 0 0 0 1];
end

%%% Calcolo delle trasformazioni tra campioni consecutivi

transforms_cam_pairs = [];
transforms_grip_pairs = [];

% Calcolo delle trasformazioni tra coppie di campioni successivi
for j = 1:num_samples-1
    current_idx = j;
    next_idx = j + 1;

    % Calcolo delle trasformazioni tra i campioni consecutivi
    % Utilizzo dell'inversa della matrice di trasformazione del campione corrente
    % per ottenere la trasformazione tra il campione corrente e il successivo
    transform_cam_pair = inv(transforms_cam_to_target(:,:,current_idx)) * transforms_cam_to_target(:,:,next_idx); %#ok<*MINV>
    transforms_cam_pairs = [transforms_cam_pairs, transform_cam_pair]; %#ok<*AGROW>
    
    transform_grip_pair = inv(transforms_grip_to_base(:,:,current_idx)) * transforms_grip_to_base(:,:,next_idx);
    transforms_grip_pairs = [transforms_grip_pairs, transform_grip_pair];
end

% Calcolo della trasformazione tra il gripper e la camera con tsai
optimal_transform_grip_to_cam = tsai(transforms_grip_pairs, transforms_cam_pairs);

%%% Visualizzazione delle trasformazioni

figure;

plotRef(eye(3), [0,0,0]', 'Object Reference'); % Disegno del sistema di riferimento del target
hold on;
grid on;
title('Transformations from Object to Base');

for k = 1:num_samples
    % Disegno della trasformazione della camera rispetto al target
    plotRef(transforms_cam_to_target(1:3,1:3, k), transforms_cam_to_target(1:3, 4, k), num2str(k));
    
    % Calcolo della trasformazione del gripper rispetto al target
    transform_target_to_grip = transforms_cam_to_target(:,:,k) * inv(optimal_transform_grip_to_cam);
    plotRef(transform_target_to_grip(1:3,1:3), transform_target_to_grip(1:3, 4), num2str(k));
    
    % Calcolo della trasformazione della base rispetto al target
    transform_target_to_base = transform_target_to_grip * inv(transforms_grip_to_base(:,:,k));
    plotRef(transform_target_to_base(1:3,1:3), transform_target_to_base(1:3, 4), 'Base');
end
