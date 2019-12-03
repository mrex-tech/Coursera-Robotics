% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
mu = [147.2244  142.5551   63.5741];
sig = [201.9200  132.5469 -218.2259;
       132.5469  128.0122 -164.0601;
      -218.2259 -164.0601  354.1857];
thre = 2.8431e-06;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 

[H, W, C] = size(I);
mask = zeros(H,W);
segI = false(H,W);
for i=1:H
    for j =1:W
        prob = mvnpdf(double([I(i,j,1) I(i,j,2) I(i,j,3)]),mu,sig);
        if prob > thre
            mask(i,j) = 1;
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.
CC = bwconncomp(mask);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
segI(CC.PixelIdxList{idx}) = true; 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;

% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
