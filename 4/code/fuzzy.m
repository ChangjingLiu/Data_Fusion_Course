clear;clc;close all;
% 观测点数
N = 500;
x = linspace(-2*pi, 2*pi,N );
% 信号
y = 3 * sin(x)+2*cos(x.^2);
% 噪声（方差1.25）
v1 = 0.05*randn(N , 1).*y';
v2 = sqrt(0.5)*randn(N , 1);
v=v1+v2;%噪声信号
z=y'+v;%真实输出信号
d=y';%期望输出信号
P=x;
T=z';

%% 画图
plot(x,y);
hold on;
plot(x,T);
axis([-2*pi 2*pi -6 6]);
legend('y-x曲线','z-x曲线');
xlabel('横坐标 x');
ylabel('纵坐标 y');

figure;
plot(x,v');
axis([-2*pi 2*pi -2 2]);
legend('v-x曲线');
xlabel('横坐标 x');
ylabel('纵坐标 y');

% 数据集设置
[a,b]=dividerand([P;T],0.8,0.2); %训练集+验证集 80%，测试集20%
trainp=a(1,:);
traint=a(2,:);
testp=b(1,:);
testt=b(2,:);