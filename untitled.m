
clear;
clc;

% �����ݺ���ģ��ת����״̬�ռ�ģ��
g11 = poly2tfd([12.8],[16.7 1],0,1);
g12 = poly2tfd([6.6],[10.9 1],0,7);
g21 = poly2tfd([-18.9],[21 1],0,3);
g22 = poly2tfd([-19.4],[14.4 1],0,3);
delt = 3;
ny = 2;
imodel = tfd2mod(delt,ny,g11,g12,g21,g22);

% ���ģ��Ԥ�������
pmodel = imodel;

% ����Ԥ��ʱ�򳤶�Ϊ6������ʱ�򳤶�Ϊ2
p = 6;
m = 2;
ywt = [];
uwt = [1000 1000];
% ��������Լ���Ͳο��켣�ȿ���������
r = [0 1];
tend = 50;    % ���õķ���ʱ��Ϊ30s
ulim = [-inf -0.15 inf inf 0.1 100];
ylim = [];
% ylim = [0 0 inf inf];
[y,u] = scmpc(pmodel,imodel,ywt,uwt,m,p,tend,r,ulim,ylim);
plotall(y,u,delt)













