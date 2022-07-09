clear;clc; close all;
N = 100;
Q = 5;w=sqrt(Q)*randn(1,N);
R = 5;v=sqrt(R)*randn(1,N);
P =eye(1);
x=zeros(1,N);
s=zeros(1,N);
Xekf=zeros(1,N);
Xekf_1=zeros(1,N);
x(1,1)=0.1;
s(1,1)=0.1;
Xekf(1,1)=x(1,1);
Xekf_1(1,1)=x(1,1);
z=zeros(1,N);
% z(1)=x(1,1)^2/20+v(1);
z(1)=x(1,1)+v(1);
zpre=zeros(1,N);
zpre(1,1)=z(1);


for k = 2 : N
    % 模拟系统
    s(:,k) = 0.5 * s(:,k-1) + (2.5 * s(:,k-1) / (1 + s(:,k-1).^2)) + 8 * cos(0.4*(k-1))+4*sin((k-1)^2);
    x(:,k) = 0.5 * x(:,k-1) + (2.5 * x(:,k-1) / (1 + x(:,k-1).^2)) + 8 * cos(0.4*(k-1)) +4*sin((k-1)^2)+ w(k-1);

%     z(k) = x(:,k).^2 / 20 + v(k);
    z(k) = x(:,k)+ v(k);
end

%%% 设置网络


% net=newff(trainp,traint,[20]);
% % 指定训练参数
% net.trainFcn='trainlm';
% net.trainParam.goal=1e-7;%训练目标设置
% net.trainParam.lr=0.01;%学习率设置,应设置为较少值，太大虽然会在开始加快收敛速度，但临近最佳点时，会产生动荡，而致使无法收敛
% net.trainParam.mc=0.9;%动量因子的设置，默认为0.9

K=zeros(1,N);
% 广义卡尔曼滤波
for k = 2 : N
    %状态预测
    Xpre = 0.5*Xekf(:,k-1)+ 2.5*Xekf(:,k-1)/(1+Xekf(:,k-1).^2) + 8 * cos(0.4*(k-1))+4*sin((k-1)^2);
    
    F = 0.5 + 2.5 * (1-Xekf.^2)/((1+Xekf.^2).^2);
%     H = Xpre/10;
    H = 1;
    
    %均方差预测方程
    PP=F*P*F'+Q;
    if k~=2
        %%% 加载训练数据
        trainp=[trainp,[Xekf(k-1)]];
        traint=[traint,Kk];
        %%% 训练
        net=newff(trainp,traint,[30]);
        [net,tr]=train(net,trainp,traint);
        [outputK]=sim(net,[Xpre]);%训练的数据，根据BP得到的结果
    else
        trainp=ones(1,1);
        traint=ones(1,1);
    end
    
    %卡尔曼增益
    if k~=2
        Kk=0.5*PP*H'*(H*PP*H'+R)^(-1)+0.5*outputK;
    else
        Kk=PP*H'*(H*PP*H'+R)^(-1);
    end
    K(1,k)=Kk;
    
    %状态更新
%     zpre =Xpre.^2/20;
    zpre =Xpre;
    Xekf(k)=Xpre+Kk*(z(k)-zpre);
    
    %均方差更新
    P=PP-Kk*H*PP;
end

% 广义卡尔曼滤波 原始方法
for k = 2 : N
    %状态预测
    Xpre = 0.5*Xekf_1(:,k-1)+ 2.5*Xekf_1(:,k-1)/(1+Xekf_1(:,k-1).^2) + 8 * cos(0.4*(k-1))+4*sin((k-1)^2);
    
    F = 0.5 + 2.5 * (1-Xekf_1.^2)/((1+Xekf_1.^2).^2);
%     H = Xpre/10;
    H = 1;
    
    %均方差预测方程
    PP=F*P*F'+Q;  
    %卡尔曼增益
    Kk=PP*H'*(H*PP*H'+R)^(-1);
    %状态更新
%     zpre =Xpre.^2/20;
    zpre =Xpre;
    Xekf_1(k)=Xpre+Kk*(z(k)-zpre);
    
    %均方差更新
    P=PP-Kk*H*PP;
end



%误差分析
for i=1:N
    Err_Obs(i)=RMS(x(:,i),z(:,i));%滤波前的误差
    Err_EKF(i)=RMS(x(:,i),Xekf(:,i));%滤波后的误差
    Err_EKF1(i)=RMS(x(:,i),Xekf_1(:,i));%滤波后的误差
end
mean_Obs=mean(Err_Obs);
mean_EKF=mean(Err_EKF);
mean_EKF1=mean(Err_EKF1);
% t = 2 : N;
% figure;
% plot(t,x(1,t),'b',t,Xekf(1,t),'r*');
% legend('真实值','EKF估计值');
figure
hold on;box on;
t=(0:1:N-1);
plot(t,s(1,:),'b','LineWidth',1);%理论轨迹
plot(t,x(1,:),'--g','LineWidth',1);%实际轨迹
plot(t,z(1,:),'-or','LineWidth',1);%观测轨迹
plot(t,Xekf(1,:),':m','LineWidth',2);%卡尔曼滤波轨迹
plot(t,Xekf_1(1,:),'-.k','LineWidth',2);%卡尔曼滤波轨迹
% M=M';
% plot(M(1,:),'k','LineWidth',1);%一步预测轨迹
legend('理论轨迹','实际运动轨迹','观测轨迹','扩展卡尔曼滤波+BP后轨迹','拓展卡尔曼');
xlabel('横坐标 T/s');
ylabel('纵坐标 X/m');

figure
hold on;box on;
plot(t,Err_Obs,'-');
plot(t,Err_EKF,'--');
plot(t,Err_EKF1,'-.');
% legend('滤波前误差',num2str(mean_Observation),'基本滤波后误差','固定增益滤波后误差');
legend(sprintf('滤波前误差%.03f',mean_Obs),sprintf('扩展卡尔曼滤波+BP后误差%.03f',mean_EKF),sprintf('扩展卡尔曼滤波后误差%.03f',mean_EKF1));
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

