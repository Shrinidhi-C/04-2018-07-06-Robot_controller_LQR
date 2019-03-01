clear
clc
load boxinfo.mat  %载入音频数据
T = data;
figure(1)
plot(T,'-*')
title('原始数据')
hold on;
%% 
%滑动平滑滤波
L = length(T);
N=10;  % 窗口大下
k = 0;
m =0 ;
for i = 1:L
    m = m+1;
    if i+N-1 > L
        break
    else
        for j = i:N+i-1
            k = k+1;
            W(k) = T(j) ;
        end
        T1(m) = mean(W);
        k = 0;
    end
end
plot(T1,'r-o')
grid
legend('原始数据','滤波之后')
