%% 匀速直线运动
clc;clear;
close all;
T=1;%雷达扫描周期
N=25/T;%总的采样次数
X=zeros(2,N);%目标真实位置、速度
X(:,1)=[0,2];%目标初始位置、速度
S(:,1)=[0,2];
Z=zeros(1,N);%传感器对位置的观测
Z(:,1)=X(1,1);%观测初始化
delta_w=1e-2; %如果增大这个参数，目标真实轨迹就是曲线了

Q=delta_w*diag([1,1.5]);%过程噪声方差
R=eye(1);%观测噪声均值

phi=[1000000,T;0,1000000];%状态转移矩阵%%%%%% 此处修改
Tau=[T^2/2,0;0,T];
u=[0;0];
W=[1;0];
H=[1,0];%观测矩阵
Xn=zeros(2,0);
Xn_fix=zeros(2,0);

for i=2:N
    S(:,i)=phi*S(:,i-1)+Tau*u;%目标理论轨迹
    X(:,i)=phi*X(:,i-1)+Tau*u+sqrtm(Q)*randn(2,1);%目标真实轨迹
    Z(:,i)=H*X(:,i)+sqrtm(R)*randn(1,1);%对目标的观测
end

% Kalman 滤波
Xkf=zeros(2,N);
Xkf(:,1)=X(:,1);%卡尔曼滤波状态初始化
Xkf_gainfix=Xkf;
Xkf_sqrt=Xkf;
P0=100e-2*eye(2);%协方差阵初始化
%% 基本离散kalman滤波
Xkf=kalman(Xkf,u,Z,H,P0,Q,R,phi,Tau,N);

%% 固定增益的kalman滤波
% Xkf_gainfix=kalman_gainfix(Xkf_gainfix,u,Z,H,P0,Q,R,phi,Tau,N);

Xkf_sqrt=kalman_sqrt(Xkf_sqrt,u,Z,H,P0,Q,R,phi,Tau,N);

%误差分析
for i=1:N

    Err_Observation(i)=RMS(X(:,i),Z(:,i));%滤波前的误差
    Err_KalmanFilter(i)=RMS(X(:,i),Xkf(:,i));%滤波后的误差
    Err_sqrtKalmanFilter(i)=RMS(X(:,i),Xkf_sqrt(:,i));%滤波后的误差
end
mean_Observation=mean(Err_Observation);
mean_KalmanFilter=mean(Err_KalmanFilter);
mean_fixKalmanFilter=mean(Err_sqrtKalmanFilter);
figure
hold on;box on;
t=(0:1:N-1);
plot(t,S(1,:),'g','LineWidth',1);%理论轨迹
plot(t,X(1,:),'--b','LineWidth',1);%实际轨迹
plot(t,Z(1,:),'-or','LineWidth',1);%观测轨迹
plot(t,Xkf(1,:),':k','LineWidth',2);%卡尔曼滤波轨迹
plot(t,Xkf_sqrt(1,:),'-.m','LineWidth',2);%卡尔曼滤波轨迹
% M=M';
% plot(M(1,:),'k','LineWidth',1);%一步预测轨迹
legend('理论轨迹','实际运动轨迹','观测轨迹','基本滤波后轨迹','平方根滤波后轨迹');
xlabel('横坐标 T/s');
ylabel('纵坐标 X/m');
 
figure
hold on;box on;
plot(t,Err_Observation,'-');
plot(t,Err_KalmanFilter,'--');
plot(t,Err_sqrtKalmanFilter,'-.');
% legend('滤波前误差',num2str(mean_Observation),'基本滤波后误差','固定增益滤波后误差');
legend(sprintf('滤波前误差%.03f',mean_Observation),sprintf('基本滤波后误差%.03f',mean_KalmanFilter),sprintf('平方根滤波后误差%.03f',mean_fixKalmanFilter));
xlabel('观测时间/s');
ylabel('误差值');


% 计算欧氏距离子函数
function dist=RMS(X1,X2)
if length(X2)<=2
    dist=sqrt((X1(1)-X2(1))^2);
else
    dist=sqrt((X1(1)-X2(1))^2);
end
end
