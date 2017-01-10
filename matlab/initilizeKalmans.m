%% Kalman filters initialization
%

T = 1/SamplingRate;

% % Constant velocity model
PHI = [1 T 0 0; 0 1 0 0; 0 0 1 T; 0 0 0 1];
H = [1 0 0 0; 0 0 1 0];

Q = [T^4/4 T^3/2 0 0; T^3/2 T^2 0 0; 0 0 T^4/4 T^3/2; 0 0 T^3/2 T^2];
R = [1 0; 0 1];

velocity_kalman = vision.KalmanFilter(PHI, H, 'ProcessNoise', Q*0.1, 'MeasurementNoise', R*1, 'StateCovariance', 1*eye(4));

%Constant position
PHI = eye(2);
H = [1 0; 0 1];

Q = [T^2 0; 0 T^2];
R = [1 0; 0 1];

position_kalman = vision.KalmanFilter(PHI, H, 'ProcessNoise', Q*0.01, 'MeasurementNoise', R*1, 'StateCovariance', 1*eye(2));

%Constant acceleration - Wiener

PHI = [1 T T^2/2 0 0 0;
    0 1 T 0 0 0;
    0 0 1 0 0 0;
    0 0 0 1 T T^2/2;
    0 0 0 0 1 T;
    0 0 0 0 0 1];

H = [1 0 0 0 0 0;
    0 0 0 1 0 0];

R = [1 0;
    0 1];

Q = [T^5/20 T^4/8 T^3/6 0 0 0;
    T^4/8 T^3/3 T^2/2 0 0 0;
    T^3/6 T^2/2 T 0 0 0;
    0 0 0 T^5/20 T^4/8 T^3/3;
    0 0 0 T^4/8 T^3/3 T^2/2;
    0 0 0 T^3/6 T^2/2 T];

acceleration_kalman = vision.KalmanFilter(PHI, H, 'ProcessNoise', Q*0.1, 'MeasurementNoise', R*1, 'StateCovariance', 1*eye(6));

    

KalmanBank{2}.filter = velocity_kalman;
KalmanBank{2}.name = 'Constant Velocity Model';

KalmanBank{1}.filter = position_kalman;
KalmanBank{1}.name = 'Constant Position Model';

KalmanBank{3}.filter = acceleration_kalman;
KalmanBank{3}.name = 'Constant Acceleration Model - Wiener';



