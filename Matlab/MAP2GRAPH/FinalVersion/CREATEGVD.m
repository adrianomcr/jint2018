function GVDMAP=CREATEGVD(InputImage,ROBOTRADIUS)
% this function uses morphological operators to find GVD or skeleton of the
% input image

global SHOWGVDFLAG;

%complement the input image
IMC_Image = imcomplement(InputImage);

% structuring element, where R specifies the radius.
% R must be a nonnegative integer. N must be 0, 4, 6, or 8
%radius of the robot
se= strel('disk', ROBOTRADIUS,0);
DILATE_Image=imdilate(IMC_Image,se);

IMC_Image2 = imcomplement(DILATE_Image);
BW=im2bw(IMC_Image2,0.5);
GVDMAP = bwmorph(BW,'skel',Inf);

if SHOWGVDFLAG==1
    subplot(2,2,1);imshow(InputImage), title('Original map');
    subplot(2,2,2);imshow(IMC_Image), title('Complement');
    subplot(2,2,3);imshow(DILATE_Image), title('Configuration free space');
    subplot(2,2,4);imshow(GVDMAP), title('GVD');
end

end