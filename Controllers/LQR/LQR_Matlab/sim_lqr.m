clear all
clc
%Sistema 
A=[0,1,0,0;0,0,-9.41492892836899,0;0,0,0,1;0,0,-192.249289283690,0];
B=[0;141.136429339344;0;1411.36429339344];
C=[1,0,0,0];
D=0;
sys=ss(A,B,C,D);
%Ganancias LQR
Q=C'*C;
R=1;
%[K,S,e]=lqr(A,B,Q,R);
K=[0.75,0.9,-33e-3,-500e-3];
NBar=-inv(C*inv(A-B*K)*B);
A2=A-B*K;
B2=NBar*B;
C2=C;
D2=0;
sys2=ss(A2,B2,C2,D2);
step(sys2)
grid
stepinfo(sys2)