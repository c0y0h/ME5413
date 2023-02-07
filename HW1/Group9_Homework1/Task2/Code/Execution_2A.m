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
    imagePath = ['sample_images\frame',No,'.jpg'];
    img = imread(imagePath); % 2A
    c_labels = semanticseg(img, net);
    cmap = camvidColorMap;
    % Fuse the categorical labels with the original image.
    output_image = labeloverlay(img,c_labels,'Colormap',cmap,'Transparency',0.4);
    % Change to your absolute file path if needed
    savePath_2A = ['C:\Users\LENOVO\Desktop\ME5413\HW1\Group9_Homework1\Task2\Results\result_2A\','result_image_',No,'.jpg'];
    imwrite(output_image,savePath_2A);
    imshow(output_image);
    pixelLabelColorbar(cmap, classes);
end