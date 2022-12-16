clear all
clc
%Sistema 
A=[0,1,0,0;0,0,-9.41492892836899,0;0,0,0,1;0,0,-192.249289283690,0];
B=[0;141.136429339344;0;1411.36429339344];
C=[0,0,1,0];
D=0;
sys=ss(A,B,C,D);
%Funcion de transferencia
[num,den]=ss2tf(A,B,C,D);
H1=tf(num,den);
%Convertir a discreto
H2=c2d(H1,0.001);
%Valores del PID
pid_values=pidtune(H2,"PID");
%pid_values=pid(4.1,70,22,0,0.001); %valores utilizados en la implementación
%Simulación
H3=feedback(series(pid_values,H2),1);
step(H3)
stepinfo(H3)