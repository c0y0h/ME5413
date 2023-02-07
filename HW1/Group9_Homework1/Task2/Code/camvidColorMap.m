function cmap = camvidColorMap()
% Define the colormap used by CamVid dataset.

cmap = [
    128 64 64     % Sky
    128 64 64     % Building
    128 64 64     % Pole
    128 64 128    % Drivable_Surface
    128 64 64     % Pavement
    128 64 64     % Tree
    128 64 64     % SignSymbol
    128 64 64     % Fence
    64 0 128      % Vehicles
    64 64 0       % Pedestrian
    0 128 192     % Bicycles_Motorcycles
    ];

% Normalize between [0 1].
cmap = cmap ./ 255;
end


