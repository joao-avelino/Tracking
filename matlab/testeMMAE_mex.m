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
filterBank(2).initialState = [1; 1; 2; 3];
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
filterBank(3).initialState = [1; 1; 2; 3; 0; 0];
filterBank(3).initialCov = 1000*eye(6);





