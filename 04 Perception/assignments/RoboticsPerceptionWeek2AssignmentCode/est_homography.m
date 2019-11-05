function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
[N, M] = size(video_pts);
A = zeros(N*2, 9);

for i=1:N
    x1  = video_pts(i, 1);
    x2  = video_pts(i, 2);
    
    x1p = logo_pts(i, 1);
    x2p = logo_pts(i, 2);
    
    A(2*i-1, :) = [-x1, -x2, -1,   0,   0,  0, x1*x1p, x2*x1p, x1p];
    A(2*i  , :) = [  0,   0,  0, -x1, -x2, -1, x1*x2p, x2*x2p, x2p];
end

[U, S, V] = svd(A);
h = V(:, 9);

H = reshape(h, 3, 3);
H = H';

end

