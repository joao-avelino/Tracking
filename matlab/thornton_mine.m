function [x,U,D] = thornton_mine(xin,Phi,Uin,Din,Gin,Q) 
% 
% function [x,U,D] = thornton(xin,Phi,Uin,Din,Gin,Q) 
% 
%  
% M. S. Grewal & A. P. Andrews 
% Kalman Filtering Theory and Practice Using MATLAB 
% Third Edition, Wiley & Sons, 2008 
%  
% 
%  Catherine Thornton's modified weighted Gram-Schmidt 
%  orthogonalization method for the predictor update of 
%  the U-D factors of the covariance matrix 
%  of estimation uncertainty in Kalman filtering 
% 
% INPUTS(with dimensions) 
%      xin(n,1) corrected estimate of state vector 
%      Phi(n,n) state transition matrix 
%      Uin(n,n) unit upper triangular factor (U) of the modified Cholesky 
%               factors (U-D factors) of the covariance matrix of 
%               corrected state estimation uncertainty (P+)  
%      Din(n,n) diagonal factor (D) of the U-D factors of the covariance 
%               matrix of corrected estimation uncertainty (P+) 
%      Gin(n,r) process noise distribution matrix (modified, if necessary to 
%               make the associated process noise covariance diagonal) 
%      Q(r,r)   diagonal covariance matrix of process noise 
%               in the stochastic system model 
% OUTPUTS: 
%      x(n,1)  predicted estimate of state vector 
%      U(n,n)  unit upper triangular factor (U) of the modified Cholesky 
%              factors (U-D factors) of the covariance matrix of 
%              predicted state estimation uncertainty (P-)  
%      D(n,n)  diagonal factor (D) of the U-D factors of the covariance 
%              matrix of predicted estimation uncertainty (P-) 
% 
x     = Phi*xin;   % state update 
[n,r] = size(Gin); % get dimensions of state(n) and process noise (r) 
G     = Gin;       % move to internal array for destructive updates 
U     = eye(n);    % initialize lower triangular part of U 
PhiU  = Phi*Uin;   % rows of [PhiU,G] are to be orthononalized 
for i=n:-1:1, 
   sigma = 0; 
   
   sigma = PhiU(i,:).^2*diag(Din);
   sigma = sigma+G(i,:).^2*diag(Q);
   
   D(i,i) = sigma; 
   for j=1:i-1, 
      sigma = 0; 
      sigma = PhiU(i,:).*diag(Din)'*PhiU(j,:)';
      sigma = sigma+G(i,:).*diag(Q)'*G(j,:)';
      
      U(j,i) = sigma/D(i,i); 
      PhiU(j,:) = PhiU(j,:)-U(j,i)*PhiU(i,:);
      % 
      G(j,:)=G(j,:)-U(j,i)*G(i,:);
   end; 
end;

