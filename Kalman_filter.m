%============== Kalman filtering ================
clear;
close all;
 
 
%=============== generate data ====================
orginal_data = (1:2:200);
noise = randn(1, 100);
data = orginal_data + noise*0.05;
 
 
%=============== design Kalman filtering ====================
% ��ز���
X = [0; 0];         % k-1 ʱ�̵�ϵͳ״̬
P = [1 0; 0 1];     % k-1 ʱ�̵�ϵͳ״̬��Ӧ��Э�������
A = [1 1; 0 1];     % ϵͳ����
Q = [0.0001 0; 0 0.0001];     % ϵͳ���̵�Э����
H = [1 0];          % ����ϵͳ�Ĳ���
R = 1;              % ϵͳ���̵�Э����
 
figure;
hold on;
for i = 2:100
   X_k = A*X;       % k ʱ�̵�ϵͳ״̬ 
   P_k = A*P*A' + Q;                % k ʱ�̵�ϵͳ״̬��Ӧ��Э�������
   Kg = P_k*H' / (H*P_k*H' + R);    % ����������(Kalman Gain)
   X = X_k + Kg*(data(i) - H*X_k);  % kʱ�̵�ϵͳ״̬�����Ż�����ֵ
   P = (eye(2) - Kg*H) * P_k;       % ����kʱ�̵�ϵͳ״̬��Ӧ��Э�������
   
   % �����ʾ�ٶ�
   scatter(i, data(i)-data(i-1), 20, 's', 'filled', 'red');
   scatter(i, X(2), 5);          
end
