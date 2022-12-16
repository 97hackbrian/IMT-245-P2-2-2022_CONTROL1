clear all;
clc;
%Sistema
A=[0,1,0,0;0,0,-9.41492892836899,0;0,0,0,1;0,0,-192.249289283690,0];
B=[0;141.136429339344;0;1411.36429339344];
C=[1,0,0,0];
D=0;
sys=ss(A,[B B],C,0);
%Ruido
w=0.01*randn(1,1000);
v=0.01*randn(1,1000);
%Datos del ruido
Mean_w=mean(w);
Var_w=(std(w))^2;
Mean_v=mean(v);
Var_v=(std(v))^2;
%Matriz de covarianzas
Q = (w*w'); %Q=2.7                               
R = (v*v'); %R=3.3
N=w*v';
%Kalman
[kalmf,L,P] = kalman(sys,Q,R,N);
%LQG
Q2=C'*C; %500
R2=1;
[K,S,e]=lqr(A,B,Q2,R2);
%K=[1.122,1.68,-52.76e-3,-719e-5]; %valores implementados
Nbar=-inv(C*inv(A-B*K)*B);
%stepinfo(out.output,out.t)