% Create classes
classes = [
    "Others1"
    "Others2"
    "Others3"
    "Drivable_Surface"
    "Others4"
    "Others5"
    "Others6"
    "Others7"
    "Vehicles"
    "Pedestrian"
    "Bicycles_Motorcycles"
    ];
data = load('deeplabv3plusResnet18CamVid.mat'); 
net = data.net;
for i=1:10
    No = num2str(i);
    imagePath = ['sample_images\frame',No,'.jpg'];
    I = imread(imagePath); % 2A
    C = semanticseg(I, net);
    cmap = camvidColorMap;
    % Fuse the categorical labels with the original image.
    B = labeloverlay(I,C,'Colormap',cmap,'Transparency',0.4);
    % Change to your absolute file path if needed
    savePath_2A = ['C:\Users\LENOVO\Desktop\ME5413\HW1\Group9_Homework1\Task2\Results\result_2A\','labeled_image_',No,'.jpg'];
    imwrite(B,savePath_2A);
    imshow(B);
    pixelLabelColorbar(cmap, classes);
end