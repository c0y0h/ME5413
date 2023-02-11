clc;
clear;
%% Set parameter
% Create classes
classes = [
    "Others"
    "Others"
    "Others"
    "DrivableSurface"
    "Others"
    "Others"
    "Others"
    "Others"
    "Vehicles"
    "Pedestrian"
    "Bicycles_Motorcycles"
    ];
%% Segementation
data = load('deeplabv3plusResnet18CamVid.mat'); 
net = data.net;
for i=1:10
    No = num2str(i);
    imagePath = ['sample_images\frame',No,'.jpg'];
    ownImagePath = ['own_test_images\',No,'.jpg'];
    img = imread(imagePath);

    % ----preprocess images to improve contrast----
    % resize test images to same as the size of images in dataset
    inputSize = net.Layers(1).InputSize;
    img = imresize(img,inputSize(1:2));
    % Contrast
    img = localcontrast(img);
    % Filter
    sigma = 0.15;
    alpha = 1.5;
    numLevels = 20;
    img = locallapfilt(img, sigma, alpha, 'NumIntensityLevels', numLevels);
    
    % Fuse the categorical labels with the original image.
    c_labels = semanticseg(img, net);
    cmap = camvidColorMap;
    output_image = labeloverlay(img,c_labels,'Colormap',cmap,'Transparency',0.4);
    savePath_2C = ['C:\Users\LENOVO\Desktop\ME5413\HW1\Group9_Homework1\Task2\Results\result_2C\','processed_image_',No,'.jpg'];
    imwrite(output_image,savePath_2C);
    imshow(output_image);
    pixelLabelColorbar(cmap, classes);
end