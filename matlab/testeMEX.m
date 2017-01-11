clear;

loadData

T = 1/SamplingRate;

PHI = [1 0 T 0; 0 1 0 T; 0 0 1 0; 0 0 0 1];
H = [1 0 0 0; 0 1 0 0];
R = [1 0; 0 1];
Q = [T^4/4 0 T^3/2 0; 0 T^4/4 0 T^3/2; T^3/2 0 T^2 0; 0 T^3/2 0 T^2];
x = [0; 0; 0; 0];
P = [2000 0 0 0; 0 2000 0 0; 0 0 2000 0; 0 0 0 2000];


kfOCV = OCvKF('new', PHI, H, Q, R*0.25, x, P/2000);
kfBIERMAN = BiermanKF('new', PHI, H, Q, R*0.25, x, P/2000);


%% Loop through all the data - OpenCV
disp('----- OPENCV ----');
tic
for k=1:numel(sampledPerson1XY(:, 1))
    
    location = [noisyPerson1XY(k, 1) noisyPerson1XY(k, 2)];
    
    [stateOCv, stateCovOCv] = OCvKF('predict', kfOCV);
    covHistoryPred{k} = stateCovOCv;
    
    if noisyPerson1XY(k, 1) == -1 || noisyPerson1XY(k, 2) == -1
    %[state, stateCov] = OCvKF('update_empty', kfOCV);
    else
    [state, stateCov] = OCvKF('update', kfOCV, location);
    end
    
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
for k=1:numel(sampledPerson1XY(:, 1))
    
    location = [noisyPerson1XY(k, 1) noisyPerson1XY(k, 2)];
    
    [stateBIER, stateCovBIER] = BiermanKF('predict', kfBIERMAN);
    covHistoryPred{k} = stateCovBIER;
    
    if noisyPerson1XY(k, 1) == -1 || noisyPerson1XY(k, 2) == -1
    [state, stateCov] = BiermanKF('update_empty', kfBIERMAN);
    else
    [state, stateCov] = BiermanKF('update', kfBIERMAN, location);
    end
    
    trackedLocationBIER(k,1) = state(1);
    trackedLocationBIER(k,2) = state(2);
    covHistoryPostBIER{k} = stateCovBIER;
    conditionNumberBIER(k) = cond(stateCovBIER);
end
toc
disp('-----------------------------------------')

window = figure(1);
win.sub1 = subplot(2,2,1);
win.sub2 = subplot(2,2,2);
win.sub3 = subplot(2,2,3);
win.sub4 = subplot(2,2,4);

hold(win.sub1, 'on');
plot(win.sub1, noisyPerson1XY(:, 1), 'b');
plot(win.sub1, trackedLocationOCV(:, 1), 'g', 'LineWidth',2);
legend(win.sub1, 'Noisy signal', 'Estimation');
title(win.sub1, 'x position - OPENCV');
% xlim(win.sub1, [300 600])
% ylim(win.sub1, [0 60])

hold(win.sub2, 'on');
plot(win.sub2, noisyPerson1XY(:, 2), 'b');
plot(win.sub2, trackedLocationOCV(:, 2), 'g', 'LineWidth',2);
legend(win.sub2, 'Noisy signal', 'Estimation')
title(win.sub2, 'y position - OPENCV');
% xlim(win.sub2, [300 600])
% ylim(win.sub2, [0 60])

hold(win.sub3, 'on');
plot(win.sub3, noisyPerson1XY(:, 1), 'b');
plot(win.sub3, trackedLocationBIER(:, 1), 'g', 'LineWidth',2);
legend(win.sub3, 'Noisy signal', 'Estimation');
title(win.sub3, 'x position - Bierman and Thornton');

hold(win.sub4, 'on');
plot(win.sub4, noisyPerson1XY(:, 2), 'b');
plot(win.sub4, trackedLocationBIER(:, 2), 'g', 'LineWidth',2);
legend(win.sub4, 'Noisy signal', 'Estimation')
title(win.sub4, 'y position - Bierman and Thornton');