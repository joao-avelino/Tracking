clear
% %Extrinsics
% 
%  RT = [0.5727 -0.8194 -0.0252 778.6;
%      -0.0456 -0.0012 -0.999 1133.4;
%      0.8185 0.5733 -0.0381 13221];
%  
%  
% % 
% % %Intrinsics
% K = [2454.6 0 258;
%     0 2454.6 204;
%     0 0 1];

% %Extrinsics
% 
 RT = [0.0001795079166553304, -0.9999999838810871, 3.838395257138583e-06, 0.1020004025872583;
   0.0009630317228184748, -3.665521717932663e-06, -0.9999995362781249, 0.9982196593086484;
   0.9999995201732892, 0.0001795115299099703, 0.0009630310493053962, -0.1243612102645947];
 
% %Intrinsics
 K = [346.5411269363589, 0, 320.5;
   0, 346.5411269363589, 240.5;
   0, 0, 1];

H = K*RT;

H = [H(:,1:2) H(:,4)];
H = inv(H);

disp('Maior escala para uma imagem 640x480 (Qs grandes), pessoa perto');

Qscx = 120*(2^(1/8)-1)/2;
Qscy = 30*(2^(1/8)-1);
Qsl = 0;

epsilonX = -Qsl/2 + Qsl.*rand(1,10000000);
epsilonY = -Qsl/2 + Qsl.*rand(1,10000000);
epsilonY = epsilonY -Qscy/2 + Qscy.*rand(1,10000000);
epsilonX = epsilonX -Qscx/2 + Qscx.*rand(1,10000000);

x = [0;0;1];
px = H*x;

px(1,:) = px(1,:)./px(3,:);
px(2,:) = px(2,:)./px(3,:);
lambda =  px(3,1);
px(3,:) = px(3,:)./px(3,:);
p = px(1:2)
px = repmat(px, 1, 10000000);


x_error = repmat(x, 1, 10000000);
x_error = x_error+[epsilonX; epsilonY; zeros(1, 10000000)];


px_error = H*x_error;

px_error(1,:) = px_error(1,:)./px_error(3,:);
px_error(2,:) = px_error(2,:)./px_error(3,:);

error = px_error(1:2,:)-px(1:2,:);

figure(2)
hist3(error', [100, 100]);
xlim([-1.5, 1.5])
ylim([-1.5, 1.5])

mu = [0.1306 0.1209];
sigma = [0.8069 0.7466; 0.7466 0.7136];
R = chol(sigma);
z = repmat(mu,10000000,1) + randn(10000000,2)*R;
figure(1);
hist3(z, [100 100]);

xlim([-1.5, 1.5])
ylim([-1.5, 1.5])

media = mean(error,2)
covariancia = cov(error')


disp('------------------ With approximation -------------------------');

Exapprox = (Qsl^2+Qscx^2)/12*((H(3,1)^2*px(1,1)-H(1,1)*H(3,1))/lambda^2)...
    +(Qsl^2+Qscy^2)/12*((H(3,2)^2*px(1,1)-H(1,2)*H(3,2))/lambda^2);
Eyapprox = (Qsl^2+Qscx^2)/12*((H(3,1)^2*px(2,1)-H(2,1)*H(3,1))/lambda^2)...
    +(Qsl^2+Qscy^2)/12*((H(3,2)^2*px(2,1)-H(2,2)*H(3,2))/lambda^2);

approxMean = [Exapprox; Eyapprox]*0.001

%Segundos momentos
Ex2 = (Qsl^2+Qscx^2)/12*((H(1,1)^2+3*H(3,1)^2*px(1,1)^2-4*H(1,1)*H(3,1)*px(1,1))/lambda^2)+...
    (Qsl^2+Qscy^2)/12*((H(1,2)^2+3*H(3,2)^2*px(1,1)^2-4*H(1,2)*H(3,2)*px(1,1))/lambda^2)...
    -2*(Exapprox+px(1,1))*px(1,1)+2*px(1,1)^2;
Ey2 = (Qsl^2+Qscx^2)/12*((H(2,1)^2+3*H(3,1)^2*px(2,1)^2-4*H(2,1)*H(3,1)*px(2,1))/lambda^2)...
    +(Qsl^2+Qscy^2)/12*((H(2,2)^2+3*H(3,2)^2*px(2,1)^2-4*H(2,2)*H(3,2)*px(2,1))/lambda^2)...
    -2*(Eyapprox+px(2,1))*px(2,1)+2*px(2,1)^2;

%Momentos cruzados
exey= (Qsl^2+Qscx^2)/12*((H(1,1)*H(2,1)-2*H(3,1)*(H(2,1)*px(1,1)...
    +H(1,1)*px(2,1))+3*H(3,1)^2*px(1,1)*px(2,1))/lambda^2)...
    +(Qsl^2+Qscy^2)/12*((H(1,2)*H(2,2)-2*H(3,2)*(H(2,2)*px(1,1)+H(1,2)*px(2,1))...
    +3*H(3,2)^2*px(1,1)*px(2,1))/lambda^2)+2*px(1,1)*px(2,1)...
    -(Exapprox+px(1,1))*px(2,1)-px(1,1)*(Eyapprox+px(2,1));


%Covariancia aproximada

approxCov = [Ex2-Exapprox^2  exey-Exapprox*Eyapprox; exey-Exapprox*Eyapprox  Ey2-Eyapprox^2]*(0.001)^2