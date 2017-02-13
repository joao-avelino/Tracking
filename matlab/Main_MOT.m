clear
tic

T = 1/18;
% 	//Comparison mode consts
COMP_POSITION = 1;
COMP_COLORS = 2;
COMP_COLORSANDPOSITION = 3;
COMP_FORMFEATURES = 4;
COMP_ALL = 5;

% 	//Metric consts
METRIC_MAHALANOBIS = 0;
METRIC_EUCLIDEAN = 1;
METRIC_CORRELATION = 2;
METRIC_INTERSECTION = 3;
METRIC_CHISQUARED = 4;
METRIC_HELLINGER = 5;
METRIC_EARTHMOVERSDISTANCE = 6;


 
%% PARAMS
 MAXIMUM_HEIGHT = 2.1;
 MINIMUM_HEIGHT = 0.5;
 
 numberOfFramesBeforeDestruction = 5;

 validation_gate = 9.22;
 metric_weight = 0.5; %How to weight colors and positions
 
 recognition_threshold = 0.7;
 c_learning_rate = 0.7;
 

 detection_confidence = 40;
 
 %Kalman params:
 P0 = 1000;
 const_pos_var = 0.01;
 const_vel_var = 0.01;
 const_accel_var = 0.01;
 
UNIT_CONVERSION = 0.001;
 
 
 %% FILES STUFF
 EVALMODE = 'test';                    %test or train
 DATASET = 'PETS09-S2L2';           %test: AVG-TownCentre , PETS09-S2L2 | train: PETS09-S2L1 ,  TUD-Stadtmitte
 ROOTDIR = 'C:\MOT_Datasets\3DMOT2015';
 
  
 
 imgDir = [ROOTDIR '\' EVALMODE '\' DATASET '\img1\'];
 
 if strcmp(EVALMODE, 'test') && strcmp(DATASET, 'AVG-TownCentre')
    [K, RT, ks, ps] = readCamParamsCI();
    UNIT_CONVERSION = 1;  %for AVG-TownCentre
 else
    [K, RT, ks, ps] = readCameraParameters([ROOTDIR '\' EVALMODE '\' DATASET '\maps\View_001.xml']);
 end
 
 invertedK = inv(K);
 
 image_files = dir([imgDir '*.jpg']);
 nfiles = length(image_files);
 
 % Build the camera params object
 cameraParams = cameraParameters('IntrinsicMatrix',K,...
            'RadialDistortion',ks, ...
            'TangentialDistortion', ps);
        
        
 %% Find the best parameters loops
 validation_gate = 9.22;
  
 numberOfFramesBeforeDestruction_list = [1 3 5];
 metric_weight_list = [0.3 0.6 0.9]; %How to weight colors and positions
 recognition_threshold_list = [0.3 0.6 0.9];
 c_learning_rate_list = [0.3 0.6 0.9];
 detection_confidence_list = [10 30 50];
 P0_list = [50 500];
 const_pos_var_list = [0.05 1 50];
 const_vel_var_list = [0.05 1 50];
 const_accel_var_list = [0.05 1 50];
 
 heightCorrection_list = [0.8 0.9 1.1 1.2];
 widthCorrection_list = [0.33 0.40 0.45 0.47];
 xCorrection_list = [0.95 0.98 1.1 1.2];
 yCorrection_list = [0.95 0.98 1.1 1.2];
 
 covX_list = [0.01 0.05 0.1 0.5 1 1.5];
 covY_list = [0.01 0.05 0.1 0.5 1 1.5];
 covXY_list = [0.01 0.05 0.1 0.5 1 1.5];
 
 BestMoTa = -100000;
 BestIDS = 100000;
 BestParamsIDS = [numberOfFramesBeforeDestruction...
     metric_weight recognition_threshold c_learning_rate...
     detection_confidence P0 const_pos_var const_vel_var const_accel_var];
 
 BestParams = [numberOfFramesBeforeDestruction...
     metric_weight recognition_threshold c_learning_rate...
     detection_confidence P0 const_pos_var const_vel_var const_accel_var];

%  for pp=1:size(P0_list, 2) 
%  for ww=1:size(metric_weight_list, 2)
%  for rec=1:size(recognition_threshold_list, 2)
%  for lr=1:size(c_learning_rate_list, 2)
%  for conf=1:size(detection_confidence_list, 2)
%  for ff=1:size(numberOfFramesBeforeDestruction_list, 2)
%  for cpv=1:size(const_pos_var_list, 2)
%  for cvv=1:size(const_vel_var_list, 2)
%  for cva=1:size(const_accel_var_list, 2)
% for cx=1:size(covX_list, 2)
% for cy=1:size(covY_list, 2)
% for cxy=1:size(covXY_list, 2)

%     covX = covX_list(cx);
%     covY = covY_list(cy);
%     covXY = covXY_list(cxy);

covX = 0.75;
covY = 0.622;
covXY = 0.2575;


%  numberOfFramesBeforeDestruction = numberOfFramesBeforeDestruction_list(ff);
%  metric_weight = metric_weight_list(ww);
%  recognition_threshold = recognition_threshold_list(rec);
%  c_learning_rate = c_learning_rate_list(lr);
%  detection_confidence = detection_confidence_list(conf);
%  P0 = P0_list(pp);
%  
 disp('params');
%  [numberOfFramesBeforeDestruction, metric_weight, recognition_threshold, c_learning_rate, detection_confidence, P0];
%  const_pos_var = const_pos_var_list(cpv);
%  const_vel_var = const_vel_var_list(cpv);
%  const_accel_var = const_accel_var_list(cpv);

 numberOfFramesBeforeDestruction = 2;
 metric_weight = 0.3;
 recognition_threshold = 0.9;
 c_learning_rate = 0.9;
 detection_confidence = 30;
 P0 = 500;
 const_pos_var = 0.05;
 const_vel_var = 0.05;
 const_accel_var = 0.05;
 
 heightCorrection  = 1;
 widthCorrection = 0.44;
 xCorrection = 1;
 yCorrection = 1;
     
  
 %Build the estimator structure
 
 %% Constant position model - and constant position height

filterBank(1).modelName='Constant Position';
filterBank(1).stateTransitionModel = [1 0 0;
                                      0 1 0;
                                      0 0 1];


filterBank(1).observationModel = [1 0 0;
                                  0 1 0;
                                  0 0 1];


filterBank(1).processNoiseCov = [T^2 0 0;
                                 0 T^2 0;
                                 0 0 T^2]*const_pos_var;


filterBank(1).observationNoiseCov = [0.9299 0 0;
                                     0 0.9299 0;
                                     0 0 0.5];


filterBank(1).initialState = [1; 1; 1.7];


filterBank(1).initialCov = P0*eye(3);


%% Constant velocity model with constant height
filterBank(2).modelName='Constant Velocity';
filterBank(2).stateTransitionModel = [1 0 0 T 0;
                                      0 1 0 0 T;
                                      0 0 1 0 0;
                                      0 0 0 1 0;
                                      0 0 0 0 1];


filterBank(2).observationModel = [1 0 0 0 0;
                                  0 1 0 0 0
                                  0 0 1 0 0];


filterBank(2).processNoiseCov = [T^4/4 0 0 T^3/2 0;
                                 0 T^4/4 0 0 T^3/2; 
                                 0 0 T^2 0 0;
                                 T^3/2 0 0 T^2 0; 
                                 0 T^3/2 0 0 T^2]*const_vel_var;


filterBank(2).observationNoiseCov = [0.9299 0 0;
                                     0 0.9299 0;
                                     0 0 0.5];

filterBank(2).initialState = [1; 1; 1.7; 0; 0];


filterBank(2).initialCov = P0*eye(5);



%% Constant acceleration model with constant height
filterBank(3).modelName='Constant Acceleration';
filterBank(3).stateTransitionModel = [1 0 0 T 0 T^2/2 0;
                                       0 1 0 0 T 0 T^2/2;
                                       0 0 1 0 0 0 0;
                                       0 0 0 1 0 T 0;
                                       0 0 0 0 1 0 T;
                                       0 0 0 0 0 1 0;
                                       0 0 0 0 0 0 1];
                                   
filterBank(3).observationModel = [1 0 0 0 0 0 0; 
                                  0 1 0 0 0 0 0;
                                  0 0 1 0 0 0 0];


filterBank(3).processNoiseCov = [T^5/20 0 0 T^4/8 0 T^3/6 0;
                                0 T^5/20 0 0 T^4/8 0 T^3/6;
                                0 0 T^2 0 0 0 0;
                                T^4/8 0 0 T^3/3 0 T^2/2 0;
                                0 T^4/8 0 0 T^3/3 0 T^2/2;
                                T^3/6 0 0 T^2/2 0 T 0;
                                0 T^3/6 0 0 T^2/2 0 T]*const_accel_var;
                            
filterBank(3).observationNoiseCov = [0.9299 0 0;
                                     0 0.9299 0;
                                     0 0 0.5];
                                 
filterBank(3).initialState = [1; 1; 1.7; 0; 0; 1; 2];
filterBank(3).initialCov = P0*eye(7);

 %Build the MoT object
 mot = MoT('new', filterBank, 444, c_learning_rate, recognition_threshold, COMP_COLORSANDPOSITION, METRIC_HELLINGER,...
     numberOfFramesBeforeDestruction, metric_weight);

 
 %Load detections
 detections = csvread([ROOTDIR '\' EVALMODE '\' DATASET '\det\det.txt']);
 
 
 %% Detection loop
 results = [];
for frame=1:nfiles
clear imagePoints;
clear pointsOnWorld;
clear TF1;
clear bbs;
clear TF2;
clear rects;
clear histogramList;
clear trackingPoints;
clear probabilities;
clear boundinBoxesReprojected;

frame;

%Load image
im = imread([imgDir image_files(frame).name]);
%im = undistortImage(I,cameraParams);

clear preBBS;
preBBS = detections(detections(:,1) == frame,:);

%Filter bad detections
preBBS = preBBS(preBBS(:,7) > detection_confidence,:);

bbs = [preBBS(:, 3), preBBS(:, 4), preBBS(:, 5), preBBS(:, 6), preBBS(:,7)];

imagePoints(:,1) = bbs(:,1)+bbs(:, 3)/2;
imagePoints(:,2) = bbs(:,2)+bbs(:, 4);


[ pointsOnWorld ] = calculatePointsOnWorldFrame( imagePoints, RT, K, invertedK,  bbs, UNIT_CONVERSION);

%Convert to meters... the camera parameters are in mm

pointsOnWorld = pointsOnWorld * UNIT_CONVERSION;

%Filter stupid detections

TF1 = pointsOnWorld(3,:) > MAXIMUM_HEIGHT;
TF2 = pointsOnWorld(3, :) < MINIMUM_HEIGHT;


toDelete = TF1 | TF2;

pointsOnWorld(:,toDelete) = [];
imagePoints(toDelete, :) = [];
bbs(toDelete,:) = [];
rects = int32(bbs(:,1:4));

pointsOnWorldHistory{frame} = pointsOnWorld;


[linsRect colsRect] = size(rects);
% Get Dario's color features ---------------------------

histogramList = zeros(linsRect, 444);

for i=1:linsRect
    %Each line is a BVT histogram
    beginX = rects(i,2);
    endX = rects(i,2)+rects(i,4);
    beginY = rects(i,1);
    endY = rects(i,1)+rects(i,3);
    
    if endX > size(im, 1)
        endX = size(im, 1);
    end
    if beginX <1;
       beginX = 1; 
    end
    if endY > size(im, 2)
        endY = size(im, 2);
    end
    if beginY < 1;
        beginY = 1;
    end
    
    pessoa = im(beginX:endX,beginY:endY,:);
    histogramList(i, :) = extractBVT_interface(pessoa, 10);
    
end

histogramHistory{frame} = histogramList;
% ------------------------------------------------------

means = [];
covariances = [];
if linsRect > 0
[ means, covariances ] = computeMeasurementStatistics( K, RT, imagePoints, rects, UNIT_CONVERSION, covX, covY, covXY);
end

%%%%%%%% 
%   Build the detections' struct array
%   Call the mex
%
%%%%%%%%

if (size(means, 2) ~= size(covariances, 3) || size(means, 2) ~= size(histogramList, 1) ||  size(means, 2) ~= size(pointsOnWorld, 2))
    if size(covariances, 1) ~= 0
    keyboard;
    end
end
clear cppDets;
cppDets = [];

for nn = 1:size(means, 2)
    cppDets(nn).pointsOnWorld = pointsOnWorld(:, nn);
    cppDets(nn).bvtHistogram = histogramList(nn, :);
    cppDets(nn).meanDetectionError = means(:, nn);
    cppDets(nn).covDetectionError = covariances(:, :, nn);
end

trackingPoints = MoT('processData', mot, cppDets);

 % trackingPoints = pointsOnWorld'; %Just testing this out
 %     trackingPoints = [trackingPoints ones(size(trackingPoints,1), 1)];
      trackingPoints(:,3) = trackingPoints(:,3)*2;
      
      trpts = trackingPoints;
      trpts(:, 1:3) = trpts(:, 1:3)/UNIT_CONVERSION;
  boundinBoxesReprojected = reprojectPointsAndDraw(trpts, im, RT, K, ks, ps, heightCorrection, widthCorrection, xCorrection, yCorrection);
% 
% 
 pause(0.01);
% 
 [linsBBrep, colsBBrep] = size(boundinBoxesReprojected);
% 
for ii=1:linsBBrep
   clear preResults;
   id = boundinBoxesReprojected(ii, 5);
   bb_left = boundinBoxesReprojected(ii, 1);%top left x?
   bb_top = boundinBoxesReprojected(ii, 2);%top left y?
   bb_width = boundinBoxesReprojected(ii, 3);
   bb_height = boundinBoxesReprojected(ii, 4);
   conf = 1;
   x = trackingPoints(ii, 1);
   y = trackingPoints(ii, 2);
   z = 0;
   preResults = [frame, id, bb_left, bb_top, bb_width, bb_height, conf, x, y, z];
   results = [results; preResults];
   
end

end
%% The results must be writen in the MoTChallenge res/data/[datasetname.txt] for evaluation

csvwrite(['res\data\' DATASET '.txt'], results);


%%%%%%%%%%%%%%%%%%%%% DELETE THE MOT OBJECT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% The benchmarkDir is the train directory of the downloaded challenge, containing all the training datasets
benchmarkDir = 'C:\MOT_Datasets\3DMOT2015\train\';

%% The evaluation script has 3 arguments:
%   1 - A txt that is a list of the datasets to be evaluated (file in the
%  seqmaps folder).
%   2 - A directory containing the results
%   3 - The benchmark directory
allMets = evaluateTracking('c2-train.txt', 'res\data\', benchmarkDir);

mota = allMets.bmark2d(12);
ids = allMets.bmark2d(10);

if mota > BestMoTa
    BestMoTa = mota;
    BestParams = [numberOfFramesBeforeDestruction...
     metric_weight recognition_threshold c_learning_rate...
     detection_confidence P0 const_pos_var const_vel_var const_accel_var];
 
    ExtraParams = [ heightCorrection, widthCorrection, xCorrection, yCorrection];
    BestCovs = [covX, covY, covXY]
end

% if ids < BestIDS
%     BestIDS = ids;
%     BestParamsIDS = [numberOfFramesBeforeDestruction...
%      metric_weight recognition_threshold c_learning_rate...
%      detection_confidence P0 const_pos_var const_vel_var const_accel_var];
% end
% end
% end
%  end
%  end
%  end
%  end
%  end
%  end
%  end
%  end
toc