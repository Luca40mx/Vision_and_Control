clc; clearvars; close all;
image_name = "foto_mia.jpeg";

% Funzione per elaborare l'immagine
I = rgb2gray(imread(image_name));
% imshow(I);

%% improve
I2 = imopen(I, strel('rectangle', [40, 30]));
% imshow(I2);

I3 = imadjust(I2);
% imshow(I3);

%% binarize
bw = imbinarize(I3);

if sum(bw, 'all') > numel(bw) / 2
    bw = ~bw;
end

bw = bwareaopen(bw, 2000);

bw = imfill(bw, 'holes');
% imshow(bw);

object = bwconncomp(bw, 4);

% Identifica gli oggetti e calcola le proprietà
numero_oggetti = object.NumObjects;
s = regionprops(object, "Centroid", "BoundingBox", "MajorAxisLength", "MinorAxisLength", "Area", "Eccentricity");
centroids = cat(1, s.Centroid);
boundingBoxes = cat(1, s.BoundingBox);
tabella = struct2table(s); % Converti la struttura in tabella
centri = tabella.Centroid;
diametri = mean([tabella.MajorAxisLength tabella.MinorAxisLength], 2);
raggi = diametri / 2;
aree = sort(tabella.Area, "ascend");

% Soglia per determinare la rotondità
soglia_eccentricita = 0.7; % puoi modificare questa soglia se necessario

% Visualizza l'immagine e le informazioni sugli oggetti
figure;
imshow(bw); hold on;

% Aggiungi i bounding box per tutti gli oggetti
for i = 1:numero_oggetti

    if s(i).Eccentricity < soglia_eccentricita
        % Disegna il bounding box per oggetti tondi
        rectangle('Position', boundingBoxes(i, :), 'EdgeColor', 'g', 'LineWidth', 2);
        % Disegna il cerchio per oggetti tondi
        viscircles(centri(i, :), raggi(i, :), 'EdgeColor', 'g');
    else
        % Disegna il bounding box per oggetti non tondi
        rectangle('Position', boundingBoxes(i, :), 'EdgeColor', 'r', 'LineWidth', 2);
    end

end

% Aggiungi i centroidi e le etichette
plot(centroids(:, 1), centroids(:, 2), ".", "MarkerSize", 40, "Color", "b"); hold on;

for i = 1:numero_oggetti
    text(centroids(i, 1), centroids(i, 2), num2str(" " + i), "FontSize", 20, 'Color', 'b');
end

text(100, 100, "I puntini indicano i centroidi", "FontSize", 20, "Color", "red");
