
%% 匀速直线运动
clc;clear;
close all;
T=1;%雷达扫描周期
N=50/T;%总的采样次数
X=zeros(2,N);%目标真实位置、速度
X(:,1)=[5,0];%目标初始位置、速度
S(:,1)=[5,0];
Z=zeros(1,N);%传感器对位置的观测
Z(:,1)=X(1,1);%观测初始化
delta_w=1e-2; %如果增大这个参数，目标真实轨迹就是曲线了

Q=delta_w*diag([1,0]);%过程噪声方差
R=eye(1)*0.1;%观测噪声均值

phi=[0.5,0;0,0];%状态转移矩阵
phi_1=[0.5,0;0,0];
Tau=[T^2/2 0;0,T];
u=[8;0];%加速度矩阵
u_1=[0;0];
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
Xkf_rk=Xkf;
Xkf_ff=Xkf;
Xkf_Sk=Xkf;
P0=100e-2*eye(2);%协方差阵初始化
%% 基本离散kalman滤波
Xkf=kalman(Xkf,u_1,Z,H,P0,Q,R,phi,Tau,N);

%% 限制K减小的kalman滤波
Xkf_rk=kalman_restrain_K(Xkf_rk,u_1,Z,H,P0,Q,R,phi,Tau,N);

%% 带遗忘因子的kalman滤波
Xkf_ff=kalman_forgetting_factor(Xkf_ff,u_1,Z,H,P0,Q,R,phi,Tau,N);

%% 扩大P的kalman滤波
Xkf_Sk=kalman_Sk(Xkf_Sk,u_1,Z,H,P0,Q,R,phi,Tau,N);

%误差分析
for i=1:N

    Err_Observation(i)=RMS(X(:,i),Z(:,i));%滤波前的误差
    Err_KalmanFilter(i)=RMS(X(:,i),Xkf(:,i));%滤波后的误差
    Err_rkKalmanFilter(i)=RMS(X(:,i),Xkf_rk(:,i));%滤波后的误差
    Err_ffKalmanFilter(i)=RMS(X(:,i),Xkf_ff(:,i));%滤波后的误差
    Err_adKalmanFilter(i)=RMS(X(:,i),Xkf_Sk(:,i));%滤波后的误差
end
mean_Observation=mean(Err_Observation);
mean_KalmanFilter=mean(Err_KalmanFilter);
mean_fixKalmanFilter=mean(Err_rkKalmanFilter);
mean_ffKalmanFilter=mean(Err_ffKalmanFilter);
mean_adKalmanFilter=mean(Err_adKalmanFilter);

%% 画图
figure
hold on;box on;
t=(0:1:N-1);
plot(t,S(1,:),'-','LineWidth',1);%理论轨迹
plot(t,X(1,:),'--','LineWidth',1);%实际轨迹
plot(t,Z(1,:),'-o','LineWidth',1);%观测轨迹
plot(t,Xkf(1,:),':','LineWidth',2);%卡尔曼滤波轨迹
plot(t,Xkf_rk(1,:),'-.','LineWidth',2);%卡尔曼滤波轨迹
plot(t,Xkf_ff(1,:),'-x','LineWidth',2);%卡尔曼滤波轨迹
plot(t,Xkf_Sk(1,:),'--s','LineWidth',2);%卡尔曼滤波轨迹
% M=M';
% plot(M(1,:),'k','LineWidth',1);%一步预测轨迹
legend('理论轨迹','实际运动轨迹','观测轨迹','基本滤波后轨迹','限制k减小滤波后轨迹','带遗忘因子卡尔曼滤波','扩大P的卡尔曼滤波');
xlabel('横坐标 T/s');
ylabel('纵坐标 X/m');
 
figure
hold on;box on;
plot(t,Err_Observation,'-');
plot(t,Err_KalmanFilter,'--');
plot(t,Err_rkKalmanFilter,'-.');
plot(t,Err_ffKalmanFilter,'-x');
plot(t,Err_adKalmanFilter,'-s');
% legend('滤波前误差',num2str(mean_Observation),'基本滤波后误差','固定增益滤波后误差');
legend(sprintf('滤波前误差%.03f',mean_Observation),sprintf('基本滤波后误差%.03f',mean_KalmanFilter),...
sprintf('限制k减小增益滤波后误差%.03f',mean_fixKalmanFilter),sprintf('带遗忘因子滤波后误差%.03f',mean_ffKalmanFilter),...
sprintf('扩大P的滤波后误差%.03f',mean_adKalmanFilter));
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
