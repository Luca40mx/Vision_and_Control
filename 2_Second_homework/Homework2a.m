% Crea un menu per la selezione dell'immagine
choice = menu('Seleziona l''immagine', ...
    'Immagine 1 (USB)', ...
'Immagine 2 (Accendino)');

% In base alla scelta dell'utente, carica e processa l'immagine
switch choice
    case 1
        % Carica e processa la prima immagine
        process_image('monete_&_usb.jpg');
    case 2
        % Carica e processa la seconda immagine
        process_image('monete_&_accendino.jpg');
    otherwise
        % Se l'utente annulla o chiude il menu
        disp('Operazione annullata.');
end

function process_image(image_name)
    % Funzione per elaborare l'immagine
    I = rgb2gray(imread(image_name));
    threshold = graythresh(I);
    bw = imbinarize(I, threshold);
    bw = bwareaopen(bw, 1000);
    object = bwconncomp(bw, 8);

    % Identifica gli oggetti e calcola le proprietà
    numero_oggetti = object.NumObjects;
    s = regionprops(object, "Centroid", "BoundingBox", "MajorAxisLength", "MinorAxisLength", "Area");
    centroids = cat(1, s.Centroid);
    boundingBoxes = cat(1, s.BoundingBox);
    tabella = struct2table(s); % Converti la struttura in tabella
    centri = tabella.Centroid;
    diametri = mean([tabella.MajorAxisLength tabella.MinorAxisLength], 2);
    raggi = diametri / 2;
    aree = sort(tabella.Area, "ascend");

    % Visualizza l'immagine e le informazioni sugli oggetti
    figure;
    imshow(I); hold on;

    % Aggiungi i bounding box
    for i = 1:numero_oggetti
        % Disegna il bounding box
        rectangle('Position', boundingBoxes(i, :), 'EdgeColor', 'r', 'LineWidth', 2);
    end

    % Aggiungi i centroidi e le etichette
    text(centroids(1, 1), centroids(1, 2), " 1", "FontSize", 20);

    for i = 2:numero_oggetti
        viscircles(centri(i, :), raggi(i, :)); hold on;
        plot(centroids(:, 1), centroids(:, 2), ".", "MarkerSize", 20); hold on;
        text(centroids(i, 1), centroids(i, 2), num2str(" " + i), "FontSize", 20);
    end

    % Inserisce le etichette basate sull'immagine
    if strcmp(image_name, 'monete_&_usb.jpg')
        text(100, 100, "I puntini indicano i centroidi", "FontSize", 20, "Color", "red");
        text(centroids(1, 1) -100, centroids(1, 2) + 100, num2str("Area: " + aree(3)), "FontSize", 8);
        text(centroids(2, 1) -100, centroids(2, 2) + 100, num2str("Area: " + aree(2)), "FontSize", 8);
        text(centroids(3, 1) -100, centroids(3, 2) + 100, num2str("Area: " + aree(1)), "FontSize", 8);
        text(centroids(1, 1) -100, centroids(1, 2) + 200, "USB", "FontSize", 20, "Color", "red");
        text(centroids(2, 1) -100, centroids(2, 2) + 200, "moneta grande", "FontSize", 20, "Color", "red");
        text(centroids(3, 1) -100, centroids(3, 2) + 200, "moneta piccola", "FontSize", 20, "Color", "red");
    else
        text(100, 100, "I puntini indicano i centroidi", "FontSize", 20, "Color", "red");
        text(centroids(1, 1) -50, centroids(1, 2) + 50, num2str("Area: " + aree(3)), "FontSize", 8);
        text(centroids(2, 1) -50, centroids(2, 2) + 50, num2str("Area: " + aree(2)), "FontSize", 8);
        text(centroids(3, 1) -50, centroids(3, 2) + 50, num2str("Area: " + aree(1)), "FontSize", 8);
        text(centroids(1, 1) -50, centroids(1, 2) + 100, "ACCENDINO", "FontSize", 15, "Color", "red");
        text(centroids(2, 1) -50, centroids(2, 2) + 150, "MONETA GRANDE", "FontSize", 15, "Color", "red");
        text(centroids(3, 1) -50, centroids(3, 2) + 150, "MONETA PICCOLA", "FontSize", 15, "Color", "red");
    end

end
