%% https://it.mathworks.com/help/images/correcting-nonuniform-illumination.html %%

clearvars; close all; clc;

I = imread("rice.png");
% imshow(I)

% per prima cosa c'è da rimuovere tutto quello che è in primo piano
% e lasciare solo il background.

% strel definisce un disco (in questo caso), di raggio dato che se usato in combo con imopen rimuove tutti i piccoli oggetti
% che NON POSSONO CONTENERE COMPLETAMENTE il disco di raggio dato.
se = strel("disk", 15); % strel sta per structuring element che sarebbe il disco

background = imopen(I, se);
% imshow(background)

I2 = I - background; % facendo cosi trovo i chicchi di riso
% imshow(I2)

% ora se voglio aumentare ulteriormente il contrasto tra chicchi e sfondo uso imadjust()

I3 = imadjust(I2);
% imshow(I3);

% ora binarizzo l'immagine
bw = imbinarize(I3);
% imshow(bw) % come si vede da qui c'è ancora un po' di rumore, quindi rimuoviamo il background
bw = bwareaopen(bw, 50);
% imshow(bw);

% ora identifichiamo gli oggetti
cc = bwconncomp(bw, 4)
cc.NumObjects

% visualizziamo l'oggetto numero 50 nell'immagine
grain = false(size(bw));
grain(cc.PixelIdxList{50}) = true;
imshow(grain)

% crea la label matrix di cc
labeled = labelmatrix(cc);
whos labeled;

% per vedere di vari colori
RGB_label = label2rgb(labeled, 'spring', 'c', 'shuffle');
% imshow(RGB_label);

%% Ora calcoliamo l'area dei vari chicchi
graindata = regionprops(cc, 'basic');

grain_areas = [graindata.Area];

% troviamo are del 50-esimo componente
grain_areas(50)

% visualizza il chicco con area minore
[min_area, idx] = min(grain_areas);
grain = false(size(bw));
grain(cc.PixelIdxList{idx}) = true;
% imshow(grain);

% faccio istogramma per aree
histogram(grain_areas)
title('Histogram of Rice Grain Area')
