clear all;
close all;
clc;
    

%load video file
[namefile, namefolder] = uigetfile('*.avi', 'Choose file .AVI'); 
path = [namefolder, namefile]; 
film = VideoReader(path);

% returns a template matcher System object, H
H = vision.TemplateMatcher;

%load templates
loadTemplates();

%get video file properties
width = get(film,'Width'); %width (in pixels)
heigh = get(film,'Height'); % height (in pixels)
frame_num = get(film,'numberOfFrames'); %frame number

 %create figure once 
 figure(1)
    
 %offset for template matching points calculation
 offset = floor(0.7*heigh);
 
for k=1:frame_num
        
    frame = read(film, k); %read frame
    
    L1 = rgb2gray(frame); 
    L1d = double(L1)/255; 
     
    %Apply a filter to increase contrast in grayscale image
     L1 = adapthisteq(L1d, 'Distribution', 'rayleigh', 'Alpha', 0.3);

    %global threshold using Otsu method
    lvlthr = graythresh(L1);  %current threshold value 
    L2otsu = im2bw(L1, 0.95*lvlthr);

    %miedian filter used to remove noise
    L2med = medfilt2(L2otsu);
    
    %extract subwindows
    L2sub = L2med (floor(0.7.*heigh):heigh, 1:floor(width/2)); %subwindow to detect lines
    L2sub2w = L2med (floor(0.7.*heigh):heigh, 1:floor(width)); %subwindow for skeleton algorithm (2x wider)
    L2pasy = L2med(floor(0.7*heigh):floor(0.8*heigh),floor(0.33*width):floor(0.66*width)); %subwindow for pedestrian
                                                                                           %crossing detection
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   line detection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %Hough transform. Hough() creates a transform, houghpeaks finds its
    %peaks and houghlines() returns edge points of found lines
	[H,theta,ro] = hough(L2sub, 'theta',-40:0.2:50); %last parameter = range of angles
   
    %last parameter = peak neighbourhood. Experimenting with this
    %parameters might give various results
    wierzch = houghpeaks(H,3,'NHoodSize', [11 111]); 
                                                                                       
    %find line edges
	lines = houghlines(L2sub,theta,ro, wierzch,'FillGap',10,'MinLength',20); 
    
    %matrices to store edge points of found lines
    x1 = zeros(1,numel(lines));
    y1 = zeros(1,numel(lines));
    x2 = zeros(1,numel(lines));
    y2 = zeros(1,numel(lines));
    
	for l=1:numel(lines)
        %get coordinates of 2 points: beginning and end of line
		x1(l) = lines(l).point1(1);
		y1(l) = lines(l).point1(2);
		x2(l) = lines(l).point2(1);
		y2(l) = lines(l).point2(2);     
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% template matching of preloaded templates of horizontal road signs
% and skeleton image
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %skeleton algorithm
    IM = bwmorph(L2sub2w,'skel',Inf);
    
    % dilation of image, to make skeleton image bolder
    IMskl = bwmorph(IM, 'dilate'); 
    
    IMskel(floor(0.7.*heigh):heigh, 1:floor(width)) = IMskl;
    
    %find cross correlation for templates and skeleton image
    for p=1:3     
        ccL = normxcorr2(tmpL{p},IMskl); %left
        ccSL = normxcorr2(tmpSL{p},IMskl); %straight left
        ccS1 = normxcorr2(tmpS1{p},IMskl); % straight 1
        ccS2 = normxcorr2(tmpS2{p},IMskl); % straight 2
        ccSR(:,:,p) = normxcorr2(tmpSR{p},IMskl); %straight right
        ccR(:,:,p) =  normxcorr2(tmpR{p},IMskl); %right
    end
    
    %points with max cross correlation coefficients
    
    [max_ccL,I] = max(ccL(:)); [yL,xL,I3] = ind2sub(size(ccL),I);
    [max_ccSL,I] = max(ccSL(:)); [ySL,xSL,I3] = ind2sub(size(ccSL),I);
    [max_ccS1,I] = max(ccS1(:)); [yS1,xS1,I3] = ind2sub(size(ccS1),I);
    [max_ccS2,I] = max(ccS2(:)); [yS2,xS2,I3] = ind2sub(size(ccS2),I);
    [max_ccSR,I] = max(ccSR(:)); [ySR,xSR,I3] = ind2sub(size(ccSR),I);
    [max_ccR,I] = max(ccR(:)); [yR,xR,I3] = ind2sub(size(ccR),I);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%          display images and draw lines
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      hold on;
        
      %L1 - grayscale image
      %L2med - image after Otsu thresholding with median filter applied
      %IMskel - skeleton image
      
      %display images
      imshow([L1])
      %imshow([L1 L2med IMskel]) 
      
        for l=1:numel(lines)
              %draw detected lines
              line ([x2(l),x1(l)],[y2(l)+floor(0.7.*heigh),y1(l)+floor(0.7.*heigh)])
        end        
       
        %display max cross correlation coefficients in command window
       [max_ccL max_ccSL max_ccS1 max_ccS2 max_ccSR max_ccR]            
         
        %draw a frame to detect pedestrian crossings
        middle = detect_zebra(L2pasy);
        
        %display frame, if pedestrian crossing has been detected
        if (middle ~= 0)
            rectangle('Position',[floor(0.33*width),floor(0.70*heigh),floor(0.33*width),floor(0.1*heigh)],'EdgeColor','y')
        end
                     
        %draw rectangle if road sign has been detected (if maximum cross
        %correlation coefficient for a template and skeleton image is higher
        %than threshold given in 'if statement)
        
         if (max_ccL >=0.52) %left - blue
            rectangle('Position',[xL(1)-size(tmpL{2},2),yL(1)-size(tmpL{2},1) + offset,floor(size(tmpL{2},2)),floor(size(tmpL{2},1))],'EdgeColor','b')
         end;
         if (max_ccSL >=0.52) %straight left - red
            rectangle('Position',[xSL(1)-size(tmpSL{2},2),ySL(1)-size(tmpSL{2},1) + offset,floor(size(tmpSL{2},2)),floor(size(tmpSL{2},1))],'EdgeColor','r')
         end;
         if (max_ccS1 >=0.64) %straight - green
            rectangle('Position',[xS1(1)-size(tmpS1{2},2),yS1(1)-size(tmpS1{2},1) + offset,floor(size(tmpS1{2},2)),floor(size(tmpS1{2},1))],'EdgeColor','g')
         end;   
         if (max_ccS2 >=0.64) % straight - green
            rectangle('Position',[xS2(1)-size(tmpS2{2},2),yS2(1)-size(tmpS2{2},1) + offset,floor(size(tmpS2{2},2)),floor(size(tmpS2{2},1))],'EdgeColor','g')
         end;
         if (max_ccSR >=0.52) %straight right - red
            rectangle('Position',[xSR(1)-size(tmpSR{2},2),ySR(1)-size(tmpSR{2},1) + offset,floor(size(tmpSR{2},2)),floor(size(tmpSR{2},1))],'EdgeColor','r')
         end;
         if (max_ccR >=0.52) % right - blue
            rectangle('Position',[xR(1)-size(tmpR{2},2),yR(1)-size(tmpR{2},1) + offset,floor(size(tmpR{2},2)),floor(size(tmpR{2},1))],'EdgeColor','b')
         end;
         
         hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % display current frame number and wait 0.1s
        k
        pause(0.1)    
end