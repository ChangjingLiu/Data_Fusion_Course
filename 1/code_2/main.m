clc;clear;
%% 信号产生
% 观测点数
N = 2000;
n = linspace(0, 1200, N);
% 信号
d = 10 * sin(pi * n / 128 + pi / 3);
% 噪声（方差1.25）
v = sqrt(1.25) * randn(N , 1);
% 观测样本值
x = d' + v;
%%%%%%%%%%%%%%%%%%%%%%%%%%


%构造FIR维纳滤波器
Wopt=FIR_wf(x,d,N); %FIR维纳滤波
H=IIR_wf(x,d',N);    %IIR维纳滤波
Ps=fft(x);      %原始信号的功率谱密度
S=H.*Ps;        %经过满足最小均方误差滤波
ss=ifft(S);     %信号还原

y = filter(Wopt, 1, x);
% 误差
En1 = d - y';
En2 = d - ss';
MSE1=mean(En1.^2);
MSE2=mean(En2.^2);
% 结果
figure, plot(n, d, 'r:', n, y, 'b-');
legend('FIR维纳滤波信号真值','FIR维纳滤波估计值'); title('FIR维纳滤波期望信号与滤波结果对比');
xlabel('观测点数');ylabel('信号幅度');
figure, plot(n , En1);
title('FIR维纳滤波误差曲线');
xlabel('观测点数');ylabel('误差幅度');

figure, plot(n, d, 'g:', n, ss, 'b-');
legend('IIR维纳滤波信号真值','IIR维纳滤波估计值'); title('IIR维纳滤波期望信号与滤波结果对比');
xlabel('观测点数');ylabel('信号幅度');
figure, plot(n , En2);
title('IIR维纳滤波误差曲线');
xlabel('观测点数');ylabel('误差幅度');