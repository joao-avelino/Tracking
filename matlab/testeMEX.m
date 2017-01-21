clear;

%loadData

load resPoints

person1 = zeros(2, 179);
person1(:) = inf;
person2 = zeros(2, 179);
person2(:) = inf;
person3 = zeros(2, 179);
person3(:) = inf;
person4 = zeros(2, 179);
person4(:) = inf;
person5 = zeros(2, 179);
person5(:) = inf;
person6 = zeros(2, 179);
person6(:) = inf;


for k=1:179
    
    %person 1
    if(size(resPointsOnWorld{k}, 2) >=1)
        person1(1:2,k) = resPointsOnWorld{k}(1:2, 1);
    end
    
    %person 2
    if(size(resPointsOnWorld{k}, 2) >=2)
        person2(1:2,k) = resPointsOnWorld{k}(1:2, 2);
    end
    
    %person 3
    if(size(resPointsOnWorld{k}, 2) >=3)
        person3(1:2,k) = resPointsOnWorld{k}(1:2, 3);
    end    
    
    %person 4
    if(size(resPointsOnWorld{k}, 2) >=4)
        person4(1:2,k) = resPointsOnWorld{k}(1:2, 4);
    end
    
    %person 5
    if(size(resPointsOnWorld{k}, 2) >=5)
        person5(1:2,k) = resPointsOnWorld{k}(1:2, 5);
    end   
    
    %person 6
    if(size(resPointsOnWorld{k}, 2) >=6)
        person6(1:2,k) = resPointsOnWorld{k}(1:2, 6);
    end 
    
    
    
end

SamplingRate = 18;

T = 1/SamplingRate;

% PHI = [1 0 T 0; 0 1 0 T; 0 0 1 0; 0 0 0 1];
% H = [1 0 0 0; 0 1 0 0];
% R = [1 0; 0 1];
% Q = [T^4/4 0 T^3/2 0; 0 T^4/4 0 T^3/2; T^3/2 0 T^2 0; 0 T^3/2 0 T^2];
% x = [0; 0; 0; 0];
% P = [2000 0 0 0; 0 2000 0 0; 0 0 2000 0; 0 0 0 2000];


PHI = [1 0 T 0 T^2/2 0;
       0 1 0 T 0 T^2/2;
       0 0 1 0 T 0;
       0 0 0 1 0 T;
       0 0 0 0 1 0;
       0 0 0 0 0 1];
   
H = [1 0 0 0 0 0; 0 1 0 0 0 0];
R = [1 0; 0 1];
Q = [T^5/20 0 T^4/8 0 T^3/6 0;
    0 T^5/20 0 T^4/8 0 T^3/6;
    T^4/8 0 T^3/3 0 T^2/2 0;
    0 T^4/8 0 T^3/3 0 T^2/2;
    T^3/6 0 T^2/2 0 T 0;
    0 T^3/6 0 T^2/2 0 T];

x = [0; 0; 0; 0; 0; 0];
P = eye(6)*200000;



kfOCV = OCvKF('new', PHI, H, Q*0.1, R*1, x, P);
kfBIERMAN = BiermanKF('new', PHI, H, Q*0.1, R*1, x, P);
matlabKF = vision.KalmanFilter(PHI, H, 'ProcessNoise', Q*0.1, 'MeasurementNoise', R*1, 'StateCovariance', P);


filterBank(1).modelName='Constant Acceleration';
filterBank(1).stateTransitionModel = PHI;                               
filterBank(1).observationModel = H;
filterBank(1).processNoiseCov = Q*0.1;
filterBank(1).observationNoiseCov = R*1;
filterBank(1).initialState = x;
filterBank(1).initialCov = P;

mmaEstimator = MMAE_kalman('new', filterBank);

%% Select which person want to track

noisyPerson1XY = person1';

%% Loop through all the data - OpenCV
disp('----- OPENCV ----');
tic
for k=1:numel(noisyPerson1XY(:, 1))
    
    location = [noisyPerson1XY(k, 1) noisyPerson1XY(k, 2)];
    
    [state, stateCovOCv] = OCvKF('predict', kfOCV);
    covHistoryPred{k} = stateCovOCv;
    
    if location(1) == Inf || location(2) == Inf
    %[state, stateCov] = OCvKF('update_empty', kfOCV);
    else
    [state, stateCov] = OCvKF('update', kfOCV, location);
    end
    
    stateOCVHist{k} = state;
    trackedLocationOCV(k,1) = state(1);
    trackedLocationOCV(k,2) = state(2);
    covHistoryPostOCV{k} = stateCovOCv;
    conditionNumberOCV(k) = cond(stateCovOCv);
end
toc

disp('-----------------------------')

%% Loop through all the data - Bierman and Thornton

disp('-------- Thornton ----------')
tic
for k=1:numel(noisyPerson1XY(:, 1))
    
    location = [noisyPerson1XY(k, 1) noisyPerson1XY(k, 2)];
    
    [state, stateCov] = BiermanKF('predict', kfBIERMAN);
    covHistoryPred{k} = stateCov;
    
    if location(1) == Inf || location(2) == Inf
    %[state, stateCov] = BiermanKF('update_empty', kfBIERMAN);
    else
    [state, stateCov] = BiermanKF('update', kfBIERMAN, location);
    end
    
    
    stateBIERHist{k} = state;
    trackedLocationBIER(k,1) = state(1);
    trackedLocationBIER(k,2) = state(2);
    covHistoryPostBIER{k} = stateCov;
    conditionNumberBIER(k) = cond(stateCov);
end
toc
disp('-----------------------------------------')

%% Loop through all the data - Matlab

disp('-------- Matlab ----------')
tic
for k=1:numel(noisyPerson1XY(:, 1))
    
    location = [noisyPerson1XY(k, 1) noisyPerson1XY(k, 2)];
    
    predict(matlabKF);
    covHistoryPred{k} = matlabKF.StateCovariance;
    
    if location(1) == Inf || location(2) == Inf
    %
    else
    correct(matlabKF,location);
    end
    
    
    stateMATLABHist{k} = matlabKF.State;
    trackedLocationMATLAB(k,1) = matlabKF.State(1);
    trackedLocationMATLAB(k,2) = matlabKF.State(2);
    covHistoryPostMATLAB{k} = matlabKF.StateCovariance;
    conditionNumberMATLAB(k) = cond(matlabKF.StateCovariance);
end
toc
disp('-----------------------------------------')


%% Loop through all the data - MMAE

disp('-------- MMAE ----------')
tic
for k=1:numel(noisyPerson1XY(:, 1))
    
    location = [noisyPerson1XY(k, 1) noisyPerson1XY(k, 2)];
    
    MMAE_kalman('predict', mmaEstimator);
    state = MMAE_kalman('getStatePrediction', mmaEstimator);
    stateCov = MMAE_kalman('getStateCovariancePrediction', mmaEstimator);

    covHistoryPred{k} = stateCov;
    
    if location(1) == Inf || location(2) == Inf
    %[state, stateCov] = BiermanKF('update_empty', kfBIERMAN);
    else
    MMAE_kalman('update', mmaEstimator, location);
    end
    state = MMAE_kalman('getStatePosterior', mmaEstimator);
    stateCov = MMAE_kalman('getStateCovariancePosterior', mmaEstimator);
    
    stateMMAEHist{k} = state;
    trackedLocationMMAE(k,1) = state(1);
    trackedLocationMMAE(k,2) = state(2);
    covHistoryPostMMAE{k} = stateCov;
    conditionNumberMMAE(k) = cond(stateCov);
end
toc
MMAE_kalman('delete', mmaEstimator);

disp('-----------------------------------------')

window = figure(1);
win.sub1 = subplot(3,2,1);
win.sub2 = subplot(3,2,2);
win.sub3 = subplot(3,2,3);
win.sub4 = subplot(3,2,4);
win.sub5 = subplot(3,2,5);
win.sub6 = subplot(3,2,6);

hold(win.sub1, 'on');
plot(win.sub1, noisyPerson1XY(:, 1), 'b*');
plot(win.sub1, trackedLocationOCV(:, 1), 'g', 'LineWidth',2);
legend(win.sub1, 'Noisy signal', 'Estimation');
title(win.sub1, 'x position - OPENCV');
% xlim(win.sub1, [300 600])
% ylim(win.sub1, [0 60])

hold(win.sub2, 'on');
plot(win.sub2, noisyPerson1XY(:, 2), 'b*');
plot(win.sub2, trackedLocationOCV(:, 2), 'g', 'LineWidth',2);
legend(win.sub2, 'Noisy signal', 'Estimation')
title(win.sub2, 'y position - OPENCV');
% xlim(win.sub2, [300 600])
% ylim(win.sub2, [0 60])

hold(win.sub3, 'on');
plot(win.sub3, noisyPerson1XY(:, 1), 'b*');
plot(win.sub3, trackedLocationBIER(:, 1), 'g', 'LineWidth',2);
legend(win.sub3, 'Noisy signal', 'Estimation');
title(win.sub3, 'x position - Bierman and Thornton');

hold(win.sub4, 'on');
plot(win.sub4, noisyPerson1XY(:, 2), 'b*');
plot(win.sub4, trackedLocationBIER(:, 2), 'g', 'LineWidth',2);
legend(win.sub4, 'Noisy signal', 'Estimation')
title(win.sub4, 'y position - Bierman and Thornton');

hold(win.sub5, 'on');
plot(win.sub5, noisyPerson1XY(:, 1), 'b*');
plot(win.sub5, trackedLocationMATLAB(:, 1), 'g', 'LineWidth',2);
legend(win.sub5, 'Noisy signal', 'Estimation');
title(win.sub5, 'x position - Matlab Kalman');

hold(win.sub6, 'on');
plot(win.sub6, noisyPerson1XY(:, 2), 'b*');
plot(win.sub6, trackedLocationMATLAB(:, 2), 'g', 'LineWidth',2);
legend(win.sub6, 'Noisy signal', 'Estimation')
title(win.sub6, 'y position - Matlab Kalman');