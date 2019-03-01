%============== Kalman filtering ================
clear;
close all;
 
 
%=============== generate data ====================
orginal_data = (1:2:200);
noise = randn(1, 100);
data = orginal_data + noise*0.05;
 
 
%=============== design Kalman filtering ====================
% 相关参数
X = [0; 0];         % k-1 时刻的系统状态
P = [1 0; 0 1];     % k-1 时刻的系统状态对应的协方差矩阵
A = [1 1; 0 1];     % 系统参数
Q = [0.0001 0; 0 0.0001];     % 系统过程的协方差
H = [1 0];          % 测量系统的参数
R = 1;              % 系统过程的协方差
 
figure;
hold on;
for i = 2:100
   X_k = A*X;       % k 时刻的系统状态 
   P_k = A*P*A' + Q;                % k 时刻的系统状态对应的协方差矩阵
   Kg = P_k*H' / (H*P_k*H' + R);    % 卡尔曼增益(Kalman Gain)
   X = X_k + Kg*(data(i) - H*X_k);  % k时刻的系统状态的最优化估算值
   P = (eye(2) - Kg*H) * P_k;       % 更新k时刻的系统状态对应的协方差矩阵
   
   % 纵轴表示速度
   scatter(i, data(i)-data(i-1), 20, 's', 'filled', 'red');
   scatter(i, X(2), 5);          
end
