
clear;
clc;

load('test2.mat');
Ts = 0.001;
J = 0.00004333;     %(kg*m^2)
B = 0.00009417;      %(kg*m^2/s)
Ka = 0.8268;       %(A/V)
Kt = 0.512;       %Nm/A
rg = 10/(2*pi);         %mm/rad
K = Ka*Kt*rg;   %(N*m^2/V)/1000
num = K/J;
den = [1 B/J K/J];