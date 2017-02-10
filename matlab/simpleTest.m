%% Load and initialize kalmans
clear
loadData
%loadNoisyFromCPP
initilizeKalmans

% noisyPerson1XY(300:2150, 1) = noisyPerson1XY(299, 1);
% noisyPerson1XY(300:2150, 2) = noisyPerson1XY(299, 2);

%% Main routine. Process the data

achieved = 0;

for k=1:numel(sampledPerson1XY(:, 1))
    
    location = [noisyPerson1XY(k, 1) noisyPerson1XY(k, 2)];
    
    if achieved == 0;
%        achieved = achieveSteadyState( location, KalmanBank, k );
         achieved = 1;
       if achieved == 1
           %Initilize all matrices. We have achieved stochastic steady
           %state
           for m=1:size(KalmanBank, 2)
               H = KalmanBank{m}.filter.MeasurementModel;
               Pminus = KalmanBank{m}.filter.StateCovariance;
               R = KalmanBank{m}.filter.MeasurementNoise;
               KalmanBank{m}.Ak = H*Pminus*H'+R;
               KalmanBank{m}.AkInv = inv(KalmanBank{m}.Ak);
               KalmanBank{m}.sqrtOfAkDeterminant = sqrt(det(KalmanBank{m}.Ak));
               KalmanBank{m}.Beta = 1/(2*pi*sqrt(det(KalmanBank{m}.Ak)));
               
               %And also initialize the probabilities associated with each
               %Kalman - I assume they are all equal
               KalmanBank{m}.p = 1/size(KalmanBank, 2);
               probs(m, k) = KalmanBank{m}.p;
           end
           [ xMMAE, probabilities ] = performMMAE( location, KalmanBank );
               KalmanBank{1,1}.p = probabilities(1);
               KalmanBank{1,2}.p = probabilities(2);
               KalmanBank{1,3}.p = probabilities(3);
               trackedLocation(k, 1) = xMMAE(1, 1);
               trackedLocation(k, 2) = xMMAE(1, 4);
               if probabilities(1) < 0.001
               probabilities(1) = 0.001;
               end
               probs(:, k) = probabilities;
       end
    else
    
    [ xMMAE, probabilities ] = performMMAE( location, KalmanBank );
    
    trackedLocation(k, 1) = xMMAE(1, 1);
    trackedLocation(k, 2) = xMMAE(1, 4);
    
    if probabilities(1) < 0.001
        probabilities(1) = 0.001;
    end
        if probabilities(2) < 0.001
        probabilities(2) = 0.001;
        end
        if probabilities(2) < 0.001
        probabilities(2) = 0.001;
    end
    
    KalmanBank{1,1}.p = probabilities(1);
    KalmanBank{1,2}.p = probabilities(2);
    KalmanBank{1,3}.p = probabilities(3);
   
    
    probs(:, k) = probabilities;
    
    
    
    end
end

% loadFromCPP

window = figure(1);
win.sub1 = subplot(2,2,1);
win.sub2 = subplot(2,2,2);
win.sub3 = subplot(2,2,[3,4]);


hold(win.sub1, 'on');
plot(win.sub1, sampledPerson1XY(:, 1), 'b');
plot(win.sub1, trackedLocation(:, 1), 'g');
legend(win.sub1, 'Ground Truth', 'Estimation');
title(win.sub1, 'x position');
% xlim(win.sub1, [300 600])
% ylim(win.sub1, [0 60])

hold(win.sub2, 'on');
plot(win.sub2, sampledPerson1XY(:, 2), 'b');
plot(win.sub2, trackedLocation(:, 2), 'g');
legend(win.sub2, 'Ground Truth', 'Estimation')
title(win.sub2, 'y position');
% xlim(win.sub2, [300 600])
% ylim(win.sub2, [0 60])


hold(win.sub3, 'on');
plot(win.sub3, probs(1, :), 'b');
plot(win.sub3, probs(2, :), 'r');
plot(win.sub3, probs(3, :), 'g');
legend(win.sub3, 'Constant Position', 'Constant Velocity', 'Constant Acceleration');
title(win.sub3, 'Probabilities - 1 Shared state');
% xlim(win.sub3, [300 600])
% ylim(win.sub3, [-0.5 1.5])



% figure(1)
% 
% hold on
% scatter(sampledPerson1XY(:, 1), 'b');
% scatter(noisyPerson1XY(:, 1), 'r');
% hold off

% figure(2)
% 
% hold on
% plot(sampledPerson1XY(:, 1), 'b');
% plot(trackedLocation(:, 1), 'r');
% hold off
% 
% h = scatter(NaN,NaN);
% hold on
% 
% xlim([min(sampledPerson1XY(:, 1))-20 max(sampledPerson1XY(:, 1))+20])
% ylim([min(sampledPerson1XY(:, 1))-20 max(sampledPerson1XY(:, 1))+20])
% 
% scatter(sampledPerson1XY(1, 1), sampledPerson1XY(1, 2), 'b');
% scatter(trackedLocation(1, 1), trackedLocation(1, 2), 'g');
%     
% legend('Ground Truth', 'Estimation');
% 
% for k=2:numel(sampledPerson1XY(:, 1))
%     scatter(sampledPerson1XY(k, 1), sampledPerson1XY(k, 2), 'b');
%     scatter(trackedLocation(k, 1), trackedLocation(k, 2), 'g');
%     pause(0.005);
% end
% 
% hold off
% 
% h = scatter(NaN,NaN);
% hold on
% 
% xlim([min(sampledPerson1XY(:, 1)) max(sampledPerson1XY(:, 1))])
% ylim([min(sampledPerson1XY(:, 1)) max(sampledPerson1XY(:, 1))])
% for k=1:numel(sampledPerson1XY(:, 1))
% scatter(sampledPerson1XY(k, 1), sampledPerson1XY(k, 2), 'b');
% scatter(mfilteredX(k), mfilteredY(k), 'r');
% pause(0.04);
% end

% figure(2)
% 
% hold on
% h = scatter(NaN,NaN);
% 
% xlim([min(sampledPerson1XY(:, 1)) max(sampledPerson1XY(:, 1))])
% ylim([min(sampledPerson1XY(:, 1)) max(sampledPerson1XY(:, 1))])
% 
% for k=1:numel(sampledPerson1XY(:, 1))
%    
%     scatter(noisyPerson1XY(k, 1), noisyPerson1XY(k, 2), 'g');
%     scatter(sampledPerson1XY(k, 1), sampledPerson1XY(k, 2), 'b');
% %    pause(0.04);
% end
% 
% hold off
