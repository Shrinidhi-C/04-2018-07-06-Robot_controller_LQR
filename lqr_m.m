
clear;
clc;
PlantX.J = 0.00004333;     %(kg*m^2)
PlantX.B = 0.00009417;      %(kg*m^2/s)
PlantX.Ka = 0.8268;       %(A/V)
PlantX.Kt = 0.512;       %Nm/A
PlantX.rg = 10/(2*pi);         %mm/rad
PlantX.K = PlantX.Ka*PlantX.Kt*PlantX.rg;   %(N*m^2/V)/1000
num = [PlantX.K/PlantX.J];
den = [1 PlantX.B/PlantX.J PlantX.K/PlantX.J];

% num = conv([-3*10^7], [1 -2.4*10^5 1.92*10^10]);
% den = conv([1 251.3 3.948*10^5], [1 2.4*10^5 1.92*10^10]);
% num = ([1 -2.4*10^5 1.92*10^10]);
% den = ([1 2.4*10^5 1.92*10^10]);
gl = tf(num, den);
g = gl/(1);
[A B C D] = ssdata(g);
% Q = [0 0 0 0; 0 0 0 0; 0 0 10000 0; 0 0 0 1];
Q = [0.01 0; 0 0.01];
R = 1;
K = lqr(A,B,Q,R);
Ac = A-B*K;Cc=C-D*K;
Gk = ss(Ac,B,Cc,D);
Gk1 = tf(Gk);
step(Gk1,'-', g, 'r--');
legend('LQR','tracking')




