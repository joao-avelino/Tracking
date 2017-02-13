function [ boundingBoxes] = reprojectPointsAndDraw( points, image, RT, K, ks, ps, heightCorrection, widthCorrection, xCorrection, yCorrection)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


if size(points, 1) == 0
   boundingBoxes = [];
   return;
end

%points -> [x, y, z, id]
%boundingBoxes -> [tl.x tl.y widht height id]
ks = [0 0];
ps = [0 0];

ids = points(:, 4)';
head = points(:, 1:3)';
[lins, cols] = size(head);

headHomogeneous = ones(4, cols);
headHomogeneous(1:3,:) = head;
feetHomogeneous = headHomogeneous;
feetHomogeneous(3,:) = 0;

% headOnImage = K*RT*headHomogeneous;
% feetOnImage = K*RT*feetHomogeneous;
% 
% denom1 = repmat(headOnImage(end, :), [lins, 1]);
% headOnImage = headOnImage./denom1;

R = RT(1:3, 1:3);
T = RT(1:3, 4);

distCoefs = [ks, ps];

headOnImage = cv.projectPoints(headHomogeneous(1:3, :)', R, T, K, 'DistCoeffs', distCoefs);
feetOnImage = cv.projectPoints(feetHomogeneous(1:3, :)', R, T, K, 'DistCoeffs', distCoefs);
headOnImage = headOnImage';
feetOnImage = feetOnImage';

heights = (feetOnImage(2,:) - headOnImage(2,:))*heightCorrection;
width = heights*widthCorrection; %Depende do aspect ratio que queremos
topLeftX = headOnImage(1,:) - width/2;
topLeftY = headOnImage(2,:);

boundingBoxes = [topLeftX; topLeftY; width; heights; ids];
boundingBoxes = boundingBoxes';

boundingBoxes(boundingBoxes<0) = 0;

figure(2)
imshow(image);
hold on;

for i=1:cols
    
rectangle('Position', boundingBoxes(i,1:4),...
	'EdgeColor','r', 'LineWidth', 3)
text(boundingBoxes(i, 1), boundingBoxes(i, 2), ['id=' int2str(boundingBoxes(i, 5))], 'FontSize', 20);
end

hold off
