clc; clear; close all; %#ok<*AGROW>

% Load depth and color images
depthImage = imread("0000006-000000166846.png");
colorImage = imread("0000006-000000167580.jpg");

% Giveng camera parameters
fx = 525;
fy = 525;
u0 = 319.5;
v0 = 239.5;

% Get image dimensions
[rows, cols] = size(depthImage);

% Create coordinate grids
[xGrid, yGrid] = meshgrid(1:cols, 1:rows);

% Filter out invalid depth values
validDepth = depthImage > 0 & depthImage < 1750; % manually set to take only the chair in the foreground

% Compute 3D coordinates for valid points
Zc = double(depthImage(validDepth));
Xc = (xGrid(validDepth) - u0) .* Zc / fx;
Yc = (yGrid(validDepth) - v0) .* Zc / fy;

% Create 3D point matrix
points3D = [Xc, Yc, Zc];

% Extract and normalize colors
R = double(colorImage(:, :, 1));
G = double(colorImage(:, :, 2));
B = double(colorImage(:, :, 3));
colors = [R(validDepth), G(validDepth), B(validDepth)] / 255;

% Plot 3D points with colors
figure;
scatter3(Xc, Yc, Zc, 6, colors, '.');
title('3D Point Cloud with Colors');
xlabel('X');
ylabel('Y');
zlabel('Z');
%% create the mesh

%% Create Mesh
% Note: This can be slow. The results are stored in ply_data.mat for convenience.

% Variables
V = points3D; % 3D points
ind = reshape(1:rows * cols, rows, cols); % Index grid
validDepth_ind = ind(validDepth); % Indices of valid points

% Initialize face array
F = []; % List of faces (triangles)

% Loop through each grid cell
for i = 1:cols - 1

    for j = 1:rows - 1

        % Indices for the corners of the grid cell
        ind_bl = sub2ind([rows cols], j + 1, i);
        ind_br = sub2ind([rows cols], j + 1, i + 1);
        ind_ul = sub2ind([rows cols], j, i);
        ind_ur = sub2ind([rows cols], j, i + 1);

        % Check if the corners are valid
        if validDepth(j + 1, i) && validDepth(j + 1, i + 1) && validDepth(j, i + 1)
            % triangle connecting the bottom left, bottom right, upper left corner of the grid cell
            f = [find(validDepth_ind == ind_bl), find(validDepth_ind == ind_br), find(validDepth_ind == ind_ur)];
            F = [F; f];
        end

        if validDepth(j, i) && validDepth(j + 1, i) && validDepth(j, i + 1)
            % triangle connecting the upper left, bottom left, upper right corner of the grid cell
            f = [find(validDepth_ind == ind_ul), find(validDepth_ind == ind_bl), find(validDepth_ind == ind_ur)];
            F = [F; f];
        elseif validDepth(j, i) && validDepth(j + 1, i) && validDepth(j + 1, i + 1) && ~ismember(ind_ul, validDepth_ind)
            % triangle connecting the upper left, bottom left, bottom right corner of the grid cell
            f = [find(validDepth_ind == ind_ul), find(validDepth_ind == ind_bl), find(validDepth_ind == ind_br)];
            F = [F; f];
        elseif validDepth(j, i) && validDepth(j, i + 1) && validDepth(j + 1, i + 1) && ~ismember(ind_bl, validDepth_ind)
            % triangle connecting the upper left, bottom right, upper right corner of the grid cell
            f = [find(validDepth_ind == ind_ul), find(validDepth_ind == ind_br), find(validDepth_ind == ind_ur)];
            F = [F; f];
        end

    end

end

%% export the mesh
exportMeshToPly(V, F, colors, 'output_mesh');

%% Teacher's function! (copy and paste from the slides available on moodle)

% Function to export mesh to PLY format
function exportMeshToPly(vertices, faces, vertex_color, name)

    if (max(max(vertex_color)) <= 1.0)
        vertex_color = vertex_color .* 256;
    end

    if (size(vertex_color, 2) == 1)
        vertex_color = repmat(vertex_color, 1, 3);
    end

    vertex_color = uint8(vertex_color);
    fidply = fopen([name '.ply'], 'w');

    fprintf(fidply, 'ply\n');
    fprintf(fidply, 'format ascii 1.0\n');
    fprintf(fidply, 'element vertex %d\n', size(vertices, 1));
    fprintf(fidply, 'property float x\n');
    fprintf(fidply, 'property float y\n');
    fprintf(fidply, 'property float z\n');
    fprintf(fidply, 'property uchar red\n');
    fprintf(fidply, 'property uchar green\n');
    fprintf(fidply, 'property uchar blue\n');
    fprintf(fidply, 'element face %d\n', size(faces, 1));
    fprintf(fidply, 'property list uchar int vertex_index\n');
    fprintf(fidply, 'end_header\n');

    for i = 1:size(vertices, 1)
        fprintf(fidply, '%f %f %f %d %d %d\n', vertices(i, 1), vertices(i, 2), vertices(i, 3), ...
            vertex_color(i, 1), vertex_color(i, 2), vertex_color(i, 3));
    end

    for i = 1:size(faces, 1)
        fprintf(fidply, '3 %d %d %d\n', faces(i, 1) - 1, faces(i, 2) - 1, faces(i, 3) - 1);
    end

    fclose(fidply);
    end
