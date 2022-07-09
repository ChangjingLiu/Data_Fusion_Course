% 观测点数
N = 50;
n = linspace(0, 50-1, N);
% 信号
rn = 0.8.^(n);
R=fft(rn);
xf=sqrt(R);
s=ifft(xf);
% 噪声（方差1.25）
v = sqrt(1) * randn(N , 1);
% 观测样本值
x = s' + v;
%%%%%%%%%%%%%%%%%%%%%%%%%%
%预测N+1，即