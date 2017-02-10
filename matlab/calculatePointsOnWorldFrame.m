function [ pointsOnWorld ] = calculatePointsOnWorldFrame( imagePoints, baseLinkToCamera, K, invertedK,  bbs, UNIT_CONVERSION)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here



imagePoints = imagePoints';

[lins1 cols1] = size(imagePoints);

uns = ones(1, cols1);

homogeneousPoints = [imagePoints; uns];

  %First normalize the points
  % K^(-1)*x_cam = [R|t]*p
  
  normalizedPoints = invertedK*homogeneousPoints;

  %If pz = 0 then, we get a homography wich we can invert
  
  homography = baseLinkToCamera(:,1:2);
  homography = [homography baseLinkToCamera(:,4)];
  
  %Finally we get the points on the base frame in homogeneous coordinates
  % p = H^-1 * (K^-1 * x_cam)
  
  homogeneousP = homography\normalizedPoints;
  
  
  %Now we just get the x, y from the homogeneous coordinates and set z to 0
  %
  %          [p1x p2x p3x ... pnx]
  %p_tilde=  [p1y p2y p3y ... pny]
  %          [l_1 l_2 l_3 ... l_n]
  %

  %x = pix/l_i
  %y = piy/l_i
  
  %divide by lambda
  
  [lins cols] = size(homogeneousP);
  [linsBBS colsBBS] = size(bbs);
  
  denom = repmat(homogeneousP(end, :), [lins 1]);
  homogeneousP = homogeneousP./denom;
  
  
  %Compute person height
  P = K*baseLinkToCamera;
  
  %BUG?
  
  for i=1:linsBBS
       %Get bbcenter
      xw = homogeneousP(1, i);
      yw = homogeneousP(2, i);
      
      xc = bbs(i,1)+bbs(i,3)/2;
      yc = bbs(i,2)+bbs(i,4)/2;
      
%       first = xw*(xc/yc*P(2,1)-P(1,1));
%       second = yw*(xc/yc*P(2,2)-P(1,2));
%       third = xc/yc*P(2,4)-P(1,4);
%       
%       numZ = first+second+third;
%      denZ = P(1,3)-xc/yc*P(2,3);

       numZ = P(2,4)+P(2,1)*xw+P(2,2)*yw-yc*(P(3,4)+P(3,1)*xw+P(3,2)*yw);
      denZ = P(2,3)-P(3,3)*yc;
      

      z = -numZ/ denZ;
      
      %There is a bug on the height computation. This doesnt fix it but
      %let's me get results
%       if xc < 300
%       if z > 0.85/UNIT_CONVERSION
%          z = 0.85/UNIT_CONVERSION; 
%       end
%       end
      
      homogeneousP(3, i) =  z;              %The heights
  end
  
  
  
  pointsOnWorld = homogeneousP;
 
  
  return;
  
end

