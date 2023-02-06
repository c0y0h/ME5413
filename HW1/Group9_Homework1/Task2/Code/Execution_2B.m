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
    ownImagePath = ['own_test_images\',No,'.jpg'];
    I = imread(ownImagePath); % 2B
    C = semanticseg(I, net);
    cmap = camvidColorMap;
    % Fuse the categorical labels with the original image.
    B = labeloverlay(I,C,'Colormap',cmap,'Transparency',0.4);
    % Change to your absolute file path if needed
    savePath_2B = ['C:\Users\LENOVO\Desktop\ME5413\HW1\Group9_Homework1\Task2\Results\result_2B\','labeled_test_image_',No,'.jpg'];
    imwrite(B,savePath_2B);
    imshow(B);
    pixelLabelColorbar(cmap, classes);
end