T = 0.5;
meas = [10; 5];
PHI = [1 0 T 0; 0 1 0 T; 0 0 1 0; 0 0 0 1];
H = [1 0 0 0; 0 1 0 0];
R = [1.5083 0.4; 0.4 0.4965];
Q = [T^4/4 0 T^3/2 0; 0 T^4/4 0 T^3/2; T^3/2 0 T^2 0; 0 T^3/2 0 T^2];
x = [10; 10; 10; 10];
P = [2000 10 10 10; 10 2000 10 10; 10 10 2000 10; 10 10 10 2000];

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

