clearvars; close all; clc;

%% carico i parametri della camera e le 2 immagini
f = 525;
u0 = 319.5;
v0 = 239.5;
I_rgb = imread("0000150-000004993884.jpg");
I_depth = imread("0000150-000004972018.png");

%% calcolo gli intrinseci per fare la nuvola di punti della scena
intrinsic = cameraIntrinsics(f, [u0, v0], [size(I_depth, 1), size(I_depth, 2)]);
point_cloud = pcfromdepth(I_depth, 1, intrinsic);

%% Applica il thresholding, binarizzo e rimuovo corpi non appartenenti alla faccia
depth_thresholded = (I_depth >= 1210) & (I_depth <= 1560) & (I_depth ~= 1557); % valori presi a mano con il datacursormode
depth_thresholded = cast(depth_thresholded, "uint8"); % cast per passare da valori logical a uint8 e non avere problemi con la imbinarize dopo
BW = imbinarize(depth_thresholded);
face_binarized = bwareaopen(BW, 10000); % cosi rimuovo tutti i corpi che non appartengono alla faccia principale

%% point cloud della faccia che ci interessa
img_depth_face = double(I_depth) .* double(face_binarized);
pc_face = pcfromdepth(img_depth_face, 1, intrinsic);

%% bordi
boundaries = bwboundaries(face_binarized);
boundaries = boundaries{1};
figure(1);
imshow(face_binarized); hold on;
plot(boundaries(:, 2), boundaries(:, 1), "r-", "LineWidth", 2);

%% region props
prop = regionprops("table", face_binarized, "BoundingBox", "Area", "Centroid", "Extrema", "MajorAxisLength", "MinorAxisLength", "Orientation");

% bounding box
x_min = prop.BoundingBox(1);
y_min = prop.BoundingBox(2);
width = prop.BoundingBox(3);
height = prop.BoundingBox(4);
rectangle("Position", [x_min, y_min, width, height], 'EdgeColor', 'r', 'LineWidth', 1);

% centroide
plot(prop.Centroid(1), prop.Centroid(2), ".", "MarkerSize", 20);

% estremi
extrema = prop.Extrema{1};
plot(extrema(:, 1), extrema(:, 2), "g.", "MarkerSize", 20);

% orientamento
xc = prop.Centroid(1);
yc = prop.Centroid(2);
alfa_rad = deg2rad(prop.Orientation);
quiver(xc, yc, cos(alfa_rad), -sin(alfa_rad), "AutoScaleFactor", 50, "LineWidth", 1, "MaxHeadSize", 4);

%% centroide 3D
u_centroide = round(prop.Centroid(1));
v_centroide = round(prop.Centroid(2));
centroide = pc_face.Location(u_centroide, v_centroide, :);
centroide = centroide(1, :);


figure(2);
pcshow(pc_face); hold on;
scatter3(centroide(1), centroide(2), centroide(3), "LineWidth", 3);

%% bordi 3D
bordo_3D = zeros(size(boundaries, 1), 3);

for i = 1:1:size(boundaries)
    bordo_3D(i, :) = point_cloud.Location(boundaries(i, 1), boundaries(i, 2), :);
end

pc_bordo = pointCloud(bordo_3D);
pc_bordo.Color = uint8(zeros(pc_bordo.Count, 3)); pc_bordo.Color(:, 1) = 255; % solo per avere il bordo rosso
pcshow(pc_bordo, "MarkerSize", 100, "ColorSource", "Color");

%% plane fitting
pc_face_first = pc_face.select(pc_face.Location(:, :, 1) ~= 0);
pc_face_second = pc_face.select(pc_face.Location(:, :, 2) ~= 0);
pc_face_third = pc_face.select(pc_face.Location(:, :, 3) ~= 0);

pc_face_forFit = pointCloud([pc_face_first.Location; pc_face_second.Location; pc_face_third.Location]);

fit_plane = pcfitplane(pc_face_forFit, 1);
figure(3);
hold on;
pcshow(pc_face_forFit);
plot(fit_plane);

%% retta per il bordo sinistro
figure(6); hold on;
bordo1 = pc_bordo.Location(:, 1) <= -137;
bordo1_sx = pc_bordo.select(bordo1);
% pcshow(bordo1_sx);

best_points_sx = best_fit_3D_line(bordo1_sx.Location);
% figure(7); hold on; % LEGGIMI:  se togli questa figura e lasci solo i primo dei 2 plot qui sotto, la retta viene messa sopra la point cloud!
% plot3(best_points1(:, 1), best_points1(:, 2), best_points1(:, 3), 'r-', 'LineWidth', 2);
% plot3(pc_bordo.Location(:, 1), pc_bordo.Location(:, 2), pc_bordo.Location(:, 3), 'b.', 'MarkerSize', 8) % point cloud of the boundary!

%% retta per il bordo destro
figure(8); hold on;
bordo2 = pc_bordo.Location(:, 1) >= 300;
bordo2_dx = pc_bordo.select(bordo2);
% pcshow(bordo2_dx);

best_points_dx = best_fit_3D_line(bordo2_dx.Location);
% figure(9); hold on;
% plot3(best_points2(:, 1), best_points2(:, 2), best_points2(:, 3), 'r-', 'LineWidth', 2) % plot best fit line
% plot3(pc_bordo.Location(:, 1), pc_bordo.Location(:, 2), pc_bordo.Location(:, 3), 'b.', 'MarkerSize', 8) % point cloud of the boundary!

%% retta per il bordo in alto
figure(10); hold on;
bordo3 = pc_bordo.Location(:, 2) >= 500 & pc_bordo.Location(:, 3) >= 1530;
bordo3_alto = pc_bordo.select(bordo3);
% pcshow(bordo3_alto);

best_points_alto = best_fit_3D_line(bordo3_alto.Location);
% figure(11); hold on;
% plot3(best_points3(:, 1), best_points3(:, 2), best_points3(:, 3), 'r-', 'LineWidth', 2) % plot best fit line
% plot3(pc_bordo.Location(:, 1), pc_bordo.Location(:, 2), pc_bordo.Location(:, 3), 'b.', 'MarkerSize', 8) % point cloud of the boundary!

%% retta per il bordo in basso
figure(12); hold on;
bordo4 = pc_bordo.Location(:, 2) <= 10 & pc_bordo.Location(:, 3) <= 1220;
bordo4_basso = pc_bordo.select(bordo4);
% pcshow(bordo4_basso);

best_points_basso = best_fit_3D_line(bordo4_basso.Location);
% figure(13); hold on;
% plot3(best_points4(:, 1), best_points4(:, 2), best_points4(:, 3), 'r-', 'LineWidth', 2) % plot best fit line
% plot3(pc_bordo.Location(:, 1), pc_bordo.Location(:, 2), pc_bordo.Location(:, 3), 'b.', 'MarkerSize', 8) % point cloud of the boundary!

%% plotting all the lines on the boundary

figure(101);
pcshow(pc_face_forFit); hold on; title("All boundary lines on the point cloud of the face");
t = -2:1:2;

directing_param_sx = best_points_sx(2, :) - best_points_sx(1, :); % it's only the difference between the 2 coordinates of the point for finding the directing parameters
x_sx = best_points_sx(1, 1) + directing_param_sx(1) * t;
y_sx = best_points_sx(1, 2) + directing_param_sx(2) * t;
z_sx = best_points_sx(1, 3) + directing_param_sx(3) * t;
line(x_sx, y_sx, z_sx);

directing_param_dx = best_points_dx(2, :) - best_points_dx(1, :);
x_dx = best_points_dx(1, 1) + directing_param_dx(1) * t;
y_dx = best_points_dx(1, 2) + directing_param_dx(2) * t;
z_dx = best_points_dx(1, 3) + directing_param_dx(3) * t;
line(x_dx, y_dx, z_dx);

directing_param_alto = best_points_alto(2, :) - best_points_alto(1, :);
x_alto = best_points_alto(1, 1) + directing_param_alto(1) * t;
y_alto = best_points_alto(1, 2) + directing_param_alto(2) * t;
z_alto = best_points_alto(1, 3) + directing_param_alto(3) * t;
line(x_alto, y_alto, z_alto);

directing_param_basso = best_points_basso(2, :) - best_points_basso(1, :);
x_basso = best_points_basso(1, 1) + directing_param_basso(1) * t;
y_basso = best_points_basso(1, 2) + directing_param_basso(2) * t;
z_basso = best_points_basso(1, 3) + directing_param_basso(3) * t;
line(x_basso, y_basso, z_basso);

% line intersection
% [pax, pbx] = lines_intersection_TEST(best_points_sx(1, 1), best_points_alto(1, 1), directing_param_sx, directing_param_alto);
% [pay, pby] = lines_intersection_TEST(best_points_sx(1, 2), best_points_alto(1, 2), directing_param_sx(1, 2), directing_param_alto(1, 2));

inters_alto_sx = lines_intersection(best_points_sx(1, :), best_points_alto(1, :), directing_param_sx, directing_param_alto);
inters_alto_dx = lines_intersection(best_points_dx(1, :), best_points_alto(1, :), directing_param_dx, directing_param_alto);
inters_basso_sx = lines_intersection(best_points_sx(1, :), best_points_basso(1, :), directing_param_sx, directing_param_basso);
inters_basso_dx = lines_intersection(best_points_dx(1, :), best_points_basso(1, :), directing_param_dx, directing_param_basso);

plot3(inters_alto_sx(1, 1), inters_alto_sx(1, 2), inters_alto_sx(1, 3), ".", MarkerSize = 30)
plot3(inters_alto_dx(1, 1), inters_alto_dx(1, 2), inters_alto_dx(1, 3), ".", MarkerSize = 30)
plot3(inters_basso_sx(1, 1), inters_basso_sx(1, 2), inters_basso_sx(1, 3), ".", MarkerSize = 30)
plot3(inters_basso_dx(1, 1), inters_basso_dx(1, 2), inters_basso_dx(1, 3), ".", MarkerSize = 30)

%% fuction that find the best coordinates for the line passing through all the point of the specified boundary
function X_end = best_fit_3D_line(X)

    % Find line of best fit (with least-squares min) through X, where X is the point cloud related to one side of the boundary of the face

    X_ave = mean(X, 1); % mean; line of best fit will pass through this point
    dX = bsxfun(@minus, X, X_ave); % residuals
    N = size(X, 1); % number of points
    C = (dX' * dX) / (N - 1); % covariance matrix of X
    [R, ~] = svd(C, 0); % singular value decomposition of C; C=R*D*R'

    % End-points of a best-fit line (segment);
    x = dX * R(:, 1); % project residuals on R(:,1)
    x_min = min(x);
    x_max = max(x);
    dx = x_max - x_min;
    Xa = (x_min - 0.05 * dx) * R(:, 1)' + X_ave;
    Xb = (x_max + 0.05 * dx) * R(:, 1)' + X_ave;
    X_end = [Xa; Xb];
end

%% QUESTA FUNZIONE E' UN TEST!!!!!!
% mi serviva solo per capire come implementare al meglio la geometria che c'è dietro l'intersezione tra le rette --> vedi pdf prof per capire meglio

function [pa, pb] = lines_intersection_TEST(p1, p2, v1, v2) %#ok<*DEFNU>
    syms mua mub real
    % pi e p2 sono i punti per cui vuoi far passare la retta e quindi i punti che ti vengono fuori dalla funzione che c'è già nel codice, uno dei 2 punti va bene!
    % anche v1 e v2 li ho già perchè sono i paramentri direttori che ho già calcolato per tracciare le 4 rette sul bordo
    dot1 = dot(p1 - p2 + mua * v1 - mub * v2, v1); % questo sarà == 0 nel solve
    dot2 = dot(p1 - p2 + mua * v1 - mub * v2, v2); % anche questo sarà == 0 nel solve

    sol = solve([dot1 == 0, dot2 == 0], [mua, mub]);
    mua_ = sol.mua;
    mub_ = sol.mub;

    pa = p1 + mua_ * v1;
    pb = p2 + mub_ * v2;

end

function intersection = lines_intersection(p1, p2, v1, v2)
    syms pa_x pa_y pa_z pb_x pb_y pb_z mua mub real;
    eq1 = [pa_x pa_y pa_z] == p1 + mua * v1;
    eq2 = [pb_x pb_y pb_z] == p2 + mub * v2;
    eq3 = dot([pa_x pa_y pa_z] - [pb_x pb_y pb_z], v1) == 0;
    eq4 = dot([pa_x pa_y pa_z] - [pb_x pb_y pb_z], v2) == 0;
    sol = solve([eq1, eq2, eq3, eq4]);

    intersection = ([double(sol.pa_x) double(sol.pa_y) double(sol.pa_z)] + [double(sol.pb_x) double(sol.pb_y) double(sol.pb_z)]) / 2;

end
