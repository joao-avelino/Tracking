%loadData;
%Transform the -1 into Inf

load('data.mat')
noisyPerson1XY(noisyPerson1XY==-1) = Inf;

T = 1/15;

%% Constant position model

filterBank(1).modelName='Constant Position';
filterBank(1).stateTransitionModel = [1 0; 0 1];
filterBank(1).observationModel = [1 0; 0 1];
filterBank(1).processNoiseCov = [T^2 0; 0 T^2];
filterBank(1).observationNoiseCov = [0.9299 0; 0 0.9299];
filterBank(1).initialState = [1; 1];
filterBank(1).initialCov = 1000*eye(2);


%% Constant velocity model
filterBank(2).modelName='Constant Velocity';
filterBank(2).stateTransitionModel = [1 0 T 0; 0 1 0 T; 0 0 1 0; 0 0 0 1];
filterBank(2).observationModel = [1 0 0 0; 0 1 0 0];
filterBank(2).processNoiseCov = [T^4/4 0 T^3/2 0; 0 T^4/4 0 T^3/2; T^3/2 0 T^2 0; 0 T^3/2 0 T^2];
filterBank(2).observationNoiseCov = [0.9299 0; 0 0.9299];
filterBank(2).initialState = [1; 1; 0; 0];
filterBank(2).initialCov = 1000*eye(4);



%% Constant acceleration model
filterBank(3).modelName='Constant Acceleration';
filterBank(3).stateTransitionModel = [1 0 T 0 T^2/2 0;
                                       0 1 0 T 0 T^2/2;
                                       0 0 1 0 T 0;
                                       0 0 0 1 0 T;
                                       0 0 0 0 1 0;
                                       0 0 0 0 0 1];                               
filterBank(3).observationModel = [1 0 0 0 0 0; 0 1 0 0 0 0];
filterBank(3).processNoiseCov = [T^5/20 0 T^4/8 0 T^3/6 0;
                                0 T^5/20 0 T^4/8 0 T^3/6;
                                T^4/8 0 T^3/3 0 T^2/2 0;
                                0 T^4/8 0 T^3/3 0 T^2/2;
                                T^3/6 0 T^2/2 0 T 0;
                                0 T^3/6 0 T^2/2 0 T];
filterBank(3).observationNoiseCov = [0.9299 0; 0 0.9299];
filterBank(3).initialState = [1; 1; 0; 0; 1; 2];
filterBank(3).initialCov = 1000*eye(6);


mmaEstimator = KalmanMex('new', filterBank);


disp('-------- MMAE ----------')
tic
for k=1:numel(noisyPerson1XY(:, 1))
    
    location = [noisyPerson1XY(k, 1) noisyPerson1XY(k, 2)];
    
    KalmanMex('predict', mmaEstimator);
    state = KalmanMex('getStatePrediction', mmaEstimator);
    stateCov = KalmanMex('getStateCovariancePrediction', mmaEstimator);

    covHistoryPred{k} = stateCov;
    
    if location(1) == Inf || location(2) == Inf
    %[state, stateCov] = BiermanKF('update_empty', kfBIERMAN);
    else
    KalmanMex('update', mmaEstimator, location);
    end
    state = KalmanMex('getStatePosterior', mmaEstimator);
    stateCov = KalmanMex('getStateCovariancePosterior', mmaEstimator);
    
    stateMMAEHist{k} = state;
    trackedLocationMMAE2(k,1) = state(1);
    trackedLocationMMAE2(k,2) = state(2);
    covHistoryPostMMAE{k} = stateCov;
    conditionNumberMMAE(k) = cond(stateCov);
    
    probsVect = KalmanMex('getAllModelProbabilities', mmaEstimator);
    
    probPos(k) = probsVect(1);
    probVel(k) = probsVect(2);
    probAccel(k) = probsVect(3);
    
end
toc
KalmanMex('delete', mmaEstimator);

disp('-----------------------------------------')

window = figure(1);
win.sub1 = subplot(2,1,1);
win.sub2 = subplot(2,1,2);

hold(win.sub1, 'on');
plot(win.sub1, sampledPerson1XY(:, 1), 'b');
plot(win.sub1, trackedLocationMMAE2(:, 1), 'g');
plot(win.sub1, noisyPerson1XY(:, 1), 'r');
legend(win.sub1, 'Ground Truth', 'Estimation');
title(win.sub1, 'x position');

hold(win.sub2, 'on');
plot(win.sub2, probPos, 'b');
plot(win.sub2, probVel, 'r');
plot(win.sub2, probAccel, 'g');
legend(win.sub2, 'Constant Position', 'Constant Velocity', 'Constant Acceleration');
title(win.sub2, 'Probabilities');

