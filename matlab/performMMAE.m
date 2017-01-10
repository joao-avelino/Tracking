function [ xMMAE, probabilities ] = performMMAE( location, KalmanBank )
%performMMAE Performs Multiple Model Adaptive Estimation
%   Given a bank of Kalman Filters, performs MMAE returning the weighted
%   states and the probability associated with each model


%DEBUG

%For each kalman filter in the bank, predict and update
for n=1:size(KalmanBank, 2)
    
    if n == 3
       1==1; 
    end
    
    predict(KalmanBank{n}.filter);
    
    H = KalmanBank{n}.filter.MeasurementModel;
    Pminus = KalmanBank{n}.filter.StateCovariance;
    R = KalmanBank{n}.filter.MeasurementNoise;
    KalmanBank{n}.Ak = H*Pminus*H'+R;
    KalmanBank{n}.AkInv = inv(KalmanBank{n}.Ak);
    KalmanBank{n}.sqrtOfAkDeterminant = sqrt(det(KalmanBank{n}.Ak));
    KalmanBank{n}.Beta = 1/(2*pi*sqrt(det(KalmanBank{n}.Ak)));
    
    KalmanState{n} = KalmanBank{n}.filter.State;
    KalmanBank{n}.KalmLocation(1) = KalmanState{n}(1);
    KalmanBank{n}.KalmLocation(2) = KalmanState{n}(size(KalmanState{n}, 1)/2+1);
    if location(1, 1) == -1;
       continue;
    end
    
%     if location == [32.023659, 21.474979]
%         if n ==3
%         disp('here');
%         end
%     end
    
    correct(KalmanBank{n}.filter,location);
    KalmanState{n} = KalmanBank{n}.filter.State;
    
end

%For each kalman filter in the bank, compute the condition density function

sumOfAll = 0;
for n=1:size(KalmanBank, 2)
    Beta = KalmanBank{n}.Beta;
    residue = location - KalmanBank{n}.KalmLocation;

    KalmanBank{n}.AkInv;
    
    if location(1, 1) == -1
        continue;
    end
    
    q = residue*KalmanBank{n}.AkInv*residue';
    
    exponential = exp((-1/2)*q);
    
    densities{n} = Beta*exp((-1/2)*q);
    sumOfAll = sumOfAll + densities{n}*KalmanBank{n}.p;
    
end
% 
if sumOfAll ~= 0 %% The residue is not too big! If it is an outlier, we dont update the probabilities
    for n=1:size(KalmanBank, 2)
        KalmanBank{n}.p = densities{n}*KalmanBank{n}.p/sumOfAll;
    end
end

for n=1:size(KalmanBank, 2)
  probabilities(n) = KalmanBank{n}.p;
end

% probabilities

xMMAE = [0 0 0 0 0 0];


%Change this later. Hardcoded!

xMMAE(1) = xMMAE(1) + probabilities(1)*KalmanState{1}(1);
xMMAE(4) = xMMAE(4) + probabilities(1)*KalmanState{1}(2);

xMMAE(1) = xMMAE(1) + probabilities(2)*KalmanState{2}(1);
xMMAE(2) = xMMAE(2) + probabilities(2)*KalmanState{2}(2);
xMMAE(4) = xMMAE(4) + probabilities(2)*KalmanState{2}(3);
xMMAE(5) = xMMAE(5) + probabilities(2)*KalmanState{2}(4);

xMMAE(1) = xMMAE(1) + probabilities(3)*KalmanState{3}(1);
xMMAE(2) = xMMAE(2) + probabilities(3)*KalmanState{3}(2);
xMMAE(3) = xMMAE(3) + probabilities(3)*KalmanState{3}(3);
xMMAE(4) = xMMAE(4) + probabilities(3)*KalmanState{3}(4);
xMMAE(5) = xMMAE(5) + probabilities(3)*KalmanState{3}(5);
xMMAE(6) = xMMAE(6) + probabilities(3)*KalmanState{3}(6);

%Testing - Feedback xMMAE into the state of the filters

KalmanBank{1}.filter.State(1) = xMMAE(1);
KalmanBank{1}.filter.State(2) = xMMAE(4);

KalmanBank{2}.filter.State(1) = xMMAE(1);
KalmanBank{2}.filter.State(2) = xMMAE(2);
KalmanBank{2}.filter.State(3) = xMMAE(4);
KalmanBank{2}.filter.State(4) = xMMAE(5);

KalmanBank{3}.filter.State(1) = xMMAE(1);
KalmanBank{3}.filter.State(2) = xMMAE(2);
KalmanBank{3}.filter.State(3) = xMMAE(3);
KalmanBank{3}.filter.State(4) = xMMAE(4);
KalmanBank{3}.filter.State(5) = xMMAE(5);
KalmanBank{3}.filter.State(6) = xMMAE(6);