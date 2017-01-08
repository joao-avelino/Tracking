clear;

T = 0.5;
meas = [10; 5];
PHI = [1 0 T 0; 0 1 0 T; 0 0 1 0; 0 0 0 1];
H = [1 0 0 0; 0 1 0 0];
R = [1.5083 0.4; 0.4 0.4965];
Q = [T^4/4 0 T^3/2 0; 0 T^4/4 0 T^3/2; T^3/2 0 T^2 0; 0 T^3/2 0 T^2];
x = [10; 10; 10; 10];
P = [2000 10 10 10; 10 2000 10 10; 10 10 2000 10; 10 10 10 2000];

velocity_kalman = vision.KalmanFilter(PHI, H, 'ProcessNoise', Q, 'MeasurementNoise', R, 'StateCovariance', P, 'State', x);


[Desc, R_new] = UDFactor(R, 'true');
[Uin, Din] = UDFactor(P, 'true');

%Decorrelate Q

[Ginv, Qin] = eig(Q);

Gin = inv(Ginv);

H_new = Desc\H; %Solve instead of inv
meas_desc = Desc\meas;

[x_pred,U_out,D_out] = thornton(x,PHI,Uin,Din,Gin,Qin);

[x_pred2,U_out2,D_out2] = thornton_mine(x,PHI,Uin,Din,Gin,Qin);

test = U_out-U_out2
testD = D_out-D_out2

D_bar = zeros(size(D_out));
U_bar = zeros(size(U_out));

predict(velocity_kalman);

maxErrorPosPRED = abs(max(x_pred2-velocity_kalman.State));
maxErrorCovPRED = abs(abs(max(U_out2*D_out2*U_out2'-velocity_kalman.StateCovariance)));


%% Para cada dimensao da medida
for i=1:size(H,1)
    
    %calculos auxiliares
    f = U_out2'*H_new(i,:)';
    g = D_out2*f;
    alpha = f'*g+R_new(i, i);
    K(:,i) = zeros(1,size(x_pred, 1));
    U_bar(1,1) = 1;
    
    gamma(1) = R_new(i,i)+g(1)*f(1);
    D_bar(1,1) = D_out2(1,1)*R_new(i,i)/gamma(1);
    U_bar(1,1) = 1;
    K(1,i) = g(1);
    
    for j=2:size(U_out2, 2)
       gamma(j) = gamma(j-1)+g(j)*f(j);
       D_bar(j, j)= D_out2(j,j)*gamma(j-1)/gamma(j);
       U_bar(:,j) = U_out2(:,j)-(f(j)/gamma(j-1))*K(:,i);
       
       K(:,i) = K(:,i)+g(j)*U_out2(:,j);
    end
    
    K(:,i) = K(:,i)/alpha;
    
    x_pred2 = x_pred2 + K(:,i)*(meas_desc(i)-H_new(i,:)*x_pred2);
    U_out2 = U_bar;
    D_out2 = D_bar;
    
    
end

K1 = K; 

Post = U_out2*U_out2*D_out2'
x_ud = x_pred2

correct(velocity_kalman, meas);
maxErrorPosPOS = abs(max(x_ud-velocity_kalman.State))
maxErrorCovPOS = abs(abs(max(Post-velocity_kalman.StateCovariance)))


