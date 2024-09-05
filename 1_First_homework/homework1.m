clearvars; close all; clc;

% Load the range image
rangeImage = imread('0000006-000000166846.png');

% Given intrinsic parameters
f = 525;
u0 = 320;
v0 = 240;
k_u = 1;
k_v = 1;

% Get image dimensions
[height, width] = size(rangeImage);

% Initialize matrices to store world coordinates
X = zeros(height, width);
Y = zeros(height, width);
Z = double(rangeImage); % Assuming the range image gives the depth directly

% Compute world coordinates
for v = 1:height

    for u = 1:width
        z = Z(v, u);

        if z > 0 % Assuming depth is positive
            x = ((u - u0) * z) / (-f * k_u);
            y = ((v - v0) * z) / (-f * k_v);
            X(v, u) = x;
            Y(v, u) = y;
        end

    end

end

% Reshape matrices into vectors for point cloud
X = X(:);
Y = Y(:);
Z = Z(:);

% Remove points with zero depth (if any)
validIndices = Z > 0;
X = X(validIndices);
Y = Y(validIndices);
Z = Z(validIndices);

% Create point cloud object
ptCloud = pointCloud([X, Y, Z]);

% visualize the color image (only to facilitate the understanding of the scene)
figure(1);
imshow("0000006-000000167580.jpg");
title("Color image", "FontSize", 40, "Color", "red")
% Visualize the point cloud
figure(2);
pcshow(ptCloud);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Point Cloud from Range Image', "FontSize", 20, "Color", "red");



%% Export for visualize the point cloud in Meshlab

% Create vertices matrix
vertices = [X, Y, Z];

faces = [];

% vertex color (white), because here we have only the range image
vertex_color = repmat([255, 255, 255], size(vertices, 1), 1);

% Export to PLY
exportMeshToPly(vertices, faces, vertex_color, 'point_cloud');

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
