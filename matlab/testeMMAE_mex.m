T = 1/1

%% Constant position model

filterBank(1).modelName='Constant Position';
filterBank(1).stateTransitionModel = [1 0; 0 1];
filterBank(1).observationModel = [1 0; 0 1];
filterBank(1).processNoiseCov = [T^2 0; 0 T^2];
filterBank(1).observationNoiseCov = [0.4421 0.3476; 0.3476 0.2747];
filterBank(1).initialState = [1; 1];
filterBank(1).initialCov = 1000*eye(2);


%% Constant velocity model
filterBank(2).modelName='Constant Velocity';
filterBank(2).stateTransitionModel = [1 0 T 0; 0 1 0 T; 0 0 1 0; 0 0 0 1];
filterBank(2).observationModel = [1 0 0 0; 0 1 0 0];
filterBank(2).processNoiseCov = [T^4/4 0 T^3/2 0; 0 T^4/4 0 T^3/2; T^3/2 0 T^2 0; 0 T^3/2 0 T^2];
filterBank(2).observationNoiseCov = [0.4421 0.3476; 0.3476 0.2747];
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
filterBank(3).observationNoiseCov = [0.4421 0.3476; 0.3476 0.2747];
filterBank(3).initialState = [1; 1; 0; 0; 1; 2];
filterBank(3).initialCov = 1000*eye(6);


mmae_instance = MMAE_kalman('new', filterBank);
MMAE_kalman('predict', mmae_instance);
statePred = MMAE_kalman('getStatePrediction', mmae_instance);
covPred = MMAE_kalman('getStateCovariancePrediction', mmae_instance);
MMAE_kalman('update', mmae_instance, [1 2]);
statePost = MMAE_kalman('getStatePrediction', mmae_instance);
covPost = MMAE_kalman('getStateCovariancePrediction', mmae_instance);
MMAE_kalman('delete', mmae_instance)

%Handmade for testing
% cpPred = filterBank(1).stateTransitionModel*1000*eye(2)*filterBank(1).stateTransitionModel'+filterBank(1).processNoiseCov;
% cpPredAug = zeros(6);
% cpPredAug(1:2,1:2) = cpPred;
% 
% cvPred = filterBank(2).stateTransitionModel*filterBank(2).initialCov*filterBank(2).stateTransitionModel'+filterBank(2).processNoiseCov;
% cvPredAug = zeros(6);
% cvPredAug(1:4,1:4) = cvPred;
% 
% caPred = filterBank(3).stateTransitionModel*1000*eye(6)*filterBank(3).stateTransitionModel'+filterBank(3).processNoiseCov;
% 
% 
% mixCov = 1/3*(cpPredAug+([1 1 0 0 0 0]'-statePred)*([1 1 0 0 0 0]'-statePred)')+1/3*(cvPredAug+([1 1 0 0 0 0]'-statePred)*([1 1 0 0 0 0]'-statePred)')+1/3*(caPred+([1.5 2 1 2 1 2]'-statePred)*([1.5 2 1 2 1 2]'-statePred)')