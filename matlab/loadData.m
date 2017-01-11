%% Falta informacao temporal

clear
groundTruth = load('gt_lab_6p.txt');
personId = 3;

%%
% Header: <number of frames> <number of people> <grid width> <grid height> <step size> <first frame> <last frame>
header = [2955	6	56	56	25	0	2950];

width = header(3);
height = header(4);

%%
% Extract grid coordinates of the first person

person1Grid = groundTruth(2:size(groundTruth, 1), personId);
person1Grid = person1Grid(person1Grid>-1);

%%
% Now extract x, y from grid coords
%
person1XY(:, 2) = ceil(person1Grid/width);
person1XY(:, 1) = person1Grid-(person1XY(:, 2)-1)*width;

%%
% Interpolate to simulate a sampling rate of 18Hz (approximately what we
% get on Vizzy. Each loaded value was sampled at 1Hz...
SamplingRate = 25;
originalSize = size(person1XY(:, 1), 1);

x = linspace(1, originalSize, originalSize);
xx = linspace(1, originalSize, originalSize*SamplingRate);

sampledPerson1XY(:, 1) = spline(x, person1XY(:, 1), xx);
sampledPerson1XY(:, 2) = spline(x, person1XY(:, 2), xx);

sampledPerson1XY_after(:, 1) = sampledPerson1XY(299:1550, 1);
sampledPerson1XY_after(:, 2) = sampledPerson1XY(299:1550, 2);

 sampledPerson1XY(300:900, 1) = sampledPerson1XY(300, 1);
 sampledPerson1XY(300:900, 2) = sampledPerson1XY(300, 2);
 
 sampledPerson1XY(899:2150, 1) = sampledPerson1XY_after(:, 1);
 sampledPerson1XY(899:2150, 1) = sampledPerson1XY_after(:, 2);

%Debug!
% 


% sampledPerson1XY(:, 1) = linspace(1, 2150, 2150);
% sampledPerson1XY(:, 2) = linspace(1, 2150, 2150);
% 
% sampledPerson1XY(300:2150, 1) = sampledPerson1XY(299, 1);
% sampledPerson1XY(300:2150, 2) = sampledPerson1XY(299, 2);

% sampledPerson1XY(:, 1) = sampledPerson1XY(:, 1).^2;
% sampledPerson1XY(:, 2) = sampledPerson1XY(:, 2).^2;

%% Add noise to the person position
%

noisyPerson1XY(:, 1) = sampledPerson1XY(:, 1);
noisyPerson1XY(:, 2) = sampledPerson1XY(:, 2);

noisyPerson1XY(:, 1) = sampledPerson1XY(:, 1)+0.5*randn(size(sampledPerson1XY(:, 1), 1), 1);
noisyPerson1XY(:, 2) = sampledPerson1XY(:, 2)+0.5*randn(size(sampledPerson1XY(:, 2), 1), 1);

%Salt and pepper noise (Outliers)
% 
spnoise = zeros(size(sampledPerson1XY(:, 1), 1), 1); 
p = randperm(size(sampledPerson1XY(:, 1), 1));
sppoints = p(1:round(length(p)/5));
spnoise(sppoints) = 2*sign(sampledPerson1XY(sppoints, 1));
sppoints = p(1:round(length(p)/5));


noisyPerson1XY(:, 1) = noisyPerson1XY(:, 1) + spnoise;

aux = noisyPerson1XY(:, 1);

aux(sppoints) = -1;

noisyPerson1XY(:, 1) = aux;

spnoise = zeros(size(sampledPerson1XY(:, 1), 1), 1); 
p = randperm(size(sampledPerson1XY(:, 1), 1));
sppoints = p(1:round(length(p)/5));
spnoise(sppoints) = 2*sign(sampledPerson1XY(sppoints, 1));

noisyPerson1XY(:, 2) = noisyPerson1XY(:, 2) + spnoise;


%%
% And plot
%
% 
% h = scatter(NaN,NaN);
% hold on
% 
% xlim([min(sampledPerson1XY(:, 1)) max(sampledPerson1XY(:, 1))])
% ylim([min(sampledPerson1XY(:, 1)) max(sampledPerson1XY(:, 1))])
% 
% for k=1:numel(sampledPerson1XY(:, 1))
%    
%     set(h, 'XData', sampledPerson1XY(1:k, 1), 'YData', sampledPerson1XY(1:k, 2));
%     pause(0.04);
%     
% end
% 
% hold off