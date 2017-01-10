clear;

T = 0.5;
meas = [10; 5];
PHI = [1 0 T 0; 0 1 0 T; 0 0 1 0; 0 0 0 1];
H = [1 0 0 0; 0 1 0 0];
R = [1.5083 0.4; 0.4 0.4965];
Q = [T^4/4 0 T^3/2 0; 0 T^4/4 0 T^3/2; T^3/2 0 T^2 0; 0 T^3/2 0 T^2];
x = [10; 10; 10; 10];
P = [2000 10 10 10; 10 2000 10 10; 10 10 2000 10; 10 10 10 2000];

kfOCV = OCvKF('new', PHI, H, Q, R, x, P);
kfBIERMAN = BiermanKF('new', PHI, H, Q, R, x, P);

[stateBier, stateCovBier] = BiermanKF('predict', kfBIERMAN);
[stateOCv, stateCovOCv] = OCvKF('predict', kfOCV);

errorCovPre = stateCovBier-stateCovOCv
errorStatePre = stateBier-stateOCv

[stateBier, stateCovBier] = BiermanKF('update', kfBIERMAN, meas);
[state, stateCov] = OCvKF('update', kfOCV, meas);

errorCovPos = stateCovBier-stateCov
errorStatePos = stateBier-state
