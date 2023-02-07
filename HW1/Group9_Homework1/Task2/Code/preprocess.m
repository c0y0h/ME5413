%% Modify the contrast of images to preprocess images
function [img_processed] = preprocess(img)
    Image=double(img);
    
    R = Image(:,:,1);
    G = Image(:,:,2);
    B = Image(:,:,3);
    
%     R = zscore(R);
%     G = zscore(G);
%     B = zscore(B);

    % set parameter
    Contrast = 10;
    Average = 127;
    
    Contrast = Contrast/100*255;
    Percent = Contrast/255;
    if(Contrast>0)
        R = Average + (R - Average) * 1 / (1 - Percent) ;
        G = Average + (G - Average) * 1 / (1 - Percent) ;
        B = Average + (B - Average) * 1 / (1 - Percent) ;
    else
        R = Average + (R - Average) * (1 + Percent);
        G = Average + (G - Average) * (1 + Percent);
        B = Average + (B - Average) * (1 + Percent);
        
    end
    
    img(:,:,1) = R;
    img(:,:,2) = G;
    img(:,:,3) = B;

    img_processed = img;

end