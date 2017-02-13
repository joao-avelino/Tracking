function [ means, covariances ] = computeMeasurementStatistics( K, RT, imagePoints, rects, UNIT_CONVERSION, covX, covY, covXY)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

H = K*RT;

H = [H(:,1:2) H(:,4)];

imagePoints = imagePoints';

[lins1, cols1] = size(imagePoints);

imagePoints = [imagePoints; ones(1, cols1)];

px = H\imagePoints;

px(1,:) = px(1,:)./px(3,:);
px(2,:) = px(2,:)./px(3,:);
lambda =  px(3,:);


H = inv(H);

[linsRect colsRect] = size(rects);

for i=1:linsRect
    Qsl = 0;
    Qscx = double(rects(i, 3)*(2^(1/8)-1)/2);
    Qscy = double(rects(i, 4)*(2^(1/8)-1));
    
    Qscx = Qscx*3;
    Qscy = Qscy;
    
%     Exapprox = (Qsl^2+Qscx^2)/12*((H(3,1)^2*px(1,i)-H(1,1)*H(3,1))/lambda(i)^2)...
%     +(Qsl^2+Qscy^2)/12*((H(3,2)^2*px(1,i)-H(1,2)*H(3,2))/lambda(i)^2);
% Eyapprox = (Qsl^2+Qscx^2)/12*((H(3,1)^2*px(2,i)-H(2,1)*H(3,1))/lambda(i)^2)...
%     +(Qsl^2+Qscy^2)/12*((H(3,2)^2*px(2,i)-H(2,2)*H(3,2))/lambda(i)^2);


%%% LINEAR
    Exapprox = 0;
    Eyapprox = 0;   
    
    means(:,i) = [Exapprox; Eyapprox; 0];
    
    
%Segundos momentos
% Ex2 = (Qsl^2+Qscx^2)/12*((H(1,1)^2+3*H(3,1)^2*px(1,i)^2-4*H(1,1)*H(3,1)*px(1,i))/lambda(i)^2)+...
%     (Qsl^2+Qscy^2)/12*((H(1,2)^2+3*H(3,2)^2*px(1,i)^2-4*H(1,2)*H(3,2)*px(1,i))/lambda(i)^2)...
%     -2*(Exapprox+px(1,i))*px(1,i)+2*px(1,i)^2;
% Ey2 = (Qsl^2+Qscx^2)/12*((H(2,1)^2+3*H(3,1)^2*px(2,i)^2-4*H(2,1)*H(3,1)*px(2,i))/lambda(i)^2)...
%     +(Qsl^2+Qscy^2)/12*((H(2,2)^2+3*H(3,2)^2*px(2,i)^2-4*H(2,2)*H(3,2)*px(2,i))/lambda(i)^2)...
%     -2*(Eyapprox+px(2,i))*px(2,i)+2*px(2,i)^2;


%%%% Linear
Ex2 = (Qsl^2+Qscx^2)/12*((H(1,1)-H(3,1)*px(1,i))^2/lambda(i)^2)+(Qsl^2+Qscy^2)/12*((H(1,2)-H(3,2)*px(1,i))^2/lambda(i)^2);

Ey2 = (Qsl^2+Qscx^2)/12*((H(2,1)-H(3,1)*px(2,i))^2/lambda(i)^2)+(Qsl^2+Qscy^2)/12*((H(2,2)-H(3,2)*px(2,i))^2/lambda(i)^2);

%Momentos cruzados
% exey= (Qsl^2+Qscx^2)/12*((H(1,1)*H(2,1)-2*H(3,1)*(H(2,1)*px(1,i)...
%     +H(1,1)*px(2,i))+3*H(3,1)^2*px(1,i)*px(2,i))/lambda(i)^2)...
%     +(Qsl^2+Qscy^2)/12*((H(1,2)*H(2,2)-2*H(3,2)*(H(2,2)*px(1,i)+H(1,2)*px(2,i))...
%     +3*H(3,2)^2*px(1,i)*px(2,i))/lambda(i)^2)+2*px(1,i)*px(2,i)...
%     -(Exapprox+px(1,i))*px(2,i)-px(1,i)*(Eyapprox+px(2,i));


%%%LINEAR
exey = (Qsl^2+Qscx^2)/12*((H(1,1)-H(3,1)*px(1,i))/lambda(i))*((H(2,1)-H(3,1)*px(2,i))/lambda(i))+...
    (Qsl^2+Qscy^2)/12*((H(1,2)-H(3,2)*px(1,i))/lambda(i))*((H(2,2)-H(3,2)*px(2,i))/lambda(i));

approxCov = [Ex2-Exapprox^2  exey-Exapprox*Eyapprox 0;
             exey-Exapprox*Eyapprox  Ey2-Eyapprox^2 0;
             0 0 0.025];

approxCov=double(approxCov);

%Perguntar ao prof como arranjo isto...
if det(approxCov) < 0

   r = min(eig(approxCov));
   approxCov = approxCov-2*r*eye(size(approxCov));
   
end

if det(approxCov) < 0
    
    disp('wtf')
    
end

  approxCov = [covX covXY 0;
                 covXY covY 0;
                 0 0 2.500];

covariances(:,:,i) = approxCov;
end

           
means = double(means)*UNIT_CONVERSION; %Converting to meters
covariances = double(covariances)*(UNIT_CONVERSION)^2; %Converting to meters

end