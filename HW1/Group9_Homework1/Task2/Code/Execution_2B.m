clc;
clear;
%% Preprocess
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
    "BicyclesMotorcycles"
    ];
%% Segementation
data = load('deeplabv3plusResnet18CamVid.mat'); 
net = data.net;
for i=1:10
    No = num2str(i);
    ownImagePath = ['own_test_images\',No,'.jpg'];
    img = imread(ownImagePath); % 2B
    c_labels = semanticseg(img, net);
    cmap = camvidColorMap;
    % Fuse the categorical labels with the original image.
    output_image = labeloverlay(img,c_labels,'Colormap',cmap,'Transparency',0.4);
    % Change to your absolute file path if needed
    savePath_2B = ['C:\Users\LENOVO\Desktop\ME5413\HW1\Group9_Homework1\Task2\Results\result_2B\','labeled_test_image_',No,'.jpg'];
    imwrite(output_image,savePath_2B);
    imshow(output_image);
    pixelLabelColorbar(cmap, classes);
end