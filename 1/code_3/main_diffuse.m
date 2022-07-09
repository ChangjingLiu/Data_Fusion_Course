
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

phi=[1,T;0,1];%状态转移矩阵
phi_1=[1,0;0,1];
Tau=[T^2/2 0;0,T];
u=[0;0];
W=[1;0];
H=[1,0];%观测矩阵
Xn=zeros(2,0);
Xn_fix=zeros(2,0);

for i=2:N
    S(:,i)=phi*S(:,i-1)+Tau*u;%目标理论轨迹
%     X(:,i)=phi*X(:,i-1)+Tau*u+sqrtm(Q)*randn(2,1);%目标真实轨迹
    X(:,i)=phi*X(:,i-1)+Tau*u+sqrtm(Q)*randn(2,1);%目标真实轨迹
    Z(:,i)=H*X(:,i)+sqrtm(R)*randn(1,1);%对目标的观测
end

% Kalman 滤波
Xkf=zeros(2,N);
Xkf(:,1)=X(:,1);%卡尔曼滤波状态初始化
Xkf_gainfix=Xkf;
P0=100e-2*eye(2);%协方差阵初始化
a=0;
ee=zeros(1,N);
EE=zeros(1,N);
%% 基本离散kalman滤波
    for i=2:N
%         Xn=phi*Xkf(:,i-1)+Tau*u;%预测
        
        Xn=phi_1*Xkf(:,i-1)+Tau*u;%假设预测错误
        P1=phi_1*P0*phi_1'+Q;%预测误差协方差
        e=Z(:,i)-H*Xn;  %新息
        E=H*P0*H'+R;    %量测估计误差
        % 判断是否发散
        r=1;%储备系数，一般取1
        ee(:,i)=e'*e;
        EE(:,i)=r*trace(E);
        if(e'*e>r*trace(E))
           disp("发散");
        end
        K=P1*H'*(H*P1*H'+R)^(-1);%增益
        Xkf(:,i)=Xn+K*(Z(:,i)-H*Xn);%状态更新
        P0=(eye(2)-K*H)*P1;             %滤波误差协方差更新
    end

%% 固定增益的kalman滤波
% Xkf_gainfix=kalman_gainfix(Xkf_gainfix,u,Z,H,P0,Q,R,phi,Tau,N);

%误差分析
for i=1:N

    Err_Observation(i)=RMS(X(:,i),Z(:,i));%滤波前的误差
    Err_KalmanFilter(i)=RMS(X(:,i),Xkf(:,i));%滤波后的误差
%     Err_fixKalmanFilter(i)=RMS(X(:,i),Xkf_gainfix(:,i));%滤波后的误差
end
mean_Observation=mean(Err_Observation);
mean_KalmanFilter=mean(Err_KalmanFilter);
% mean_fixKalmanFilter=mean(Err_fixKalmanFilter);
figure
hold on;box on;
t=(0:1:N-1);
plot(t,S(1,:),'g','LineWidth',1);%理论轨迹
plot(t,X(1,:),'--b','LineWidth',1);%实际轨迹
plot(t,Z(1,:),'-or','LineWidth',1);%观测轨迹
plot(t,Xkf(1,:),':k','LineWidth',2);%卡尔曼滤波轨迹
% plot(t,Xkf_gainfix(1,:),'-.m','LineWidth',2);%卡尔曼滤波轨迹
% M=M';
% plot(M(1,:),'k','LineWidth',1);%一步预测轨迹
legend('理论轨迹','实际运动轨迹','观测轨迹','基本滤波后轨迹');
xlabel('横坐标 T/s');
ylabel('纵坐标 X/m');
 
figure
hold on;box on;
plot(t,Err_Observation,'-');
plot(t,Err_KalmanFilter,'--');
% plot(t,Err_fixKalmanFilter,'-.');
% legend('滤波前误差',num2str(mean_Observation),'基本滤波后误差','固定增益滤波后误差');
legend(sprintf('滤波前误差%.03f',mean_Observation),sprintf('基本滤波后误差%.03f',mean_KalmanFilter));
xlabel('观测时间/s');
ylabel('误差值');

figure
hold on;box on;
plot(t,ee,'-');
plot(t,EE,'--');
legend("判据左侧","判据右侧");
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
