clear;

T = 0.5;
meas = [10; 5];
PHI = [1 0 T 0; 0 1 0 T; 0 0 1 0; 0 0 0 1];
H = [1 0 0 0; 0 1 0 0];
R = [1.5083 0.4; 0.4 0.4965];
Q = [T^4/4 0 T^3/2 0; 0 T^4/4 0 T^3/2; T^3/2 0 T^2 0; 0 T^3/2 0 T^2];
x = [10; 10; 10; 10];
P = [2000 10 10 10; 10 2000 10 10; 10 10 2000 10; 10 10 10 2000];

kf = BiermanKF('new', PHI, H, Q, R, x, P);


[state, stateCov] = KalmanMex('predict', kf)
[state, stateCov] = KalmanMex('update', kf, meas)
