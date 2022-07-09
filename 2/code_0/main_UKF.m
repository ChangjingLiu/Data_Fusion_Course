%%%
clear;clc; close all;
N = 50;
Q = 1;w=sqrt(Q)*randn(1,N);
R = 1;v=sqrt(R)*randn(1,N);
P =eye(1);
X=zeros(1,N);
s=zeros(1,N);
Xukf=zeros(1,N);
X(1,1)=0.1;
s(1,1)=0.1;
Xukf(1,1)=X(1,1);
Z=zeros(1,N);
Z(1)=hfun(X(1,1))+v(1);

for k = 2 : N
    % 模拟系统
    s(:,k) = gfun(s(:,k-1),k-1);
    X(:,k) = gfun(X(:,k-1),k-1)+ w(k-1);
    Z(k) =hfun(X(:,k),k)+ v(k);
end

%UKF滤波，UT变换
L=1;        %L个变量
alpha=1;    %0~1
kalpha=0;
belta=2;    %建议为2
ramda=3-L;
for j=1:2*L+1
    Wm(j)=1/(2*(L+ramda));
    Wc(j)=1/(2*(L+ramda));
end
Wm(1)=ramda/(L+ramda);%权值Wm的初值需要另外定
Wc(1)=ramda/(L+ramda)+1-alpha^2+belta;%权值Wc的初值需要另外定
%%%%%%%%%%%%%%%%
Xukf = zeros(L,N);
Xukf(:,1) = X(:, 1);
P0=eye(L);%协方差阵初始化
for t=2:N
    xestimate=Xukf(:,t-1);%获取上一步的UKF点
    P=P0;%协方差阵
    
    %(1) 获得一组Sigma点集
    cho=(chol(P*(L+ramda)))';
    for k=1:L
        xgamaP1(:,k)=xestimate+cho(:,k);
        xgamaP2(:,k)=xestimate-cho(:,k);
    end
    Xsigma=[xestimate,xgamaP1,xgamaP2];%xestimate是上一步的点，相当于均值点
    
    %(2) 对Sigma点集进行一步预测
    Xsigmapre=gfun(Xsigma,t);
    
    %（3）计算加权均值
    Xpred=zeros(L,1);
    for k=1:2*L+1
        Xpred=Xpred+Wm(k)*Xsigmapre(:,k);%均值
    end
    %（4）计算加权方差
    Ppred=zeros(L,L);
    for k=1:2*L+1
        Ppred=Ppred+Wc(k)*(Xsigmapre(:,k)-Xpred)*(Xsigmapre(:,k)-Xpred)';%协方差矩阵
    end
    Ppred=Ppred+Q;
    
    %（5）根据预测值，再一次使用UT变换，得到新的sigma点集
    chor=(chol((L+ramda)*Ppred))';
    for k=1:L
        XaugsigmaP1(:,k)=Xpred+chor(:,k);
        XaugsigmaP2(:,k)=Xpred-chor(:,k);
    end
    Xaugsigma=[Xpred XaugsigmaP1 XaugsigmaP2];
    
    %（6）观测预测
    for k=1:2*L+1
        Zsigmapre(1,k)=hfun(Xaugsigma(:,k),k);
    end
    
    %（7）计算观测预测加权均值
    Zpred=0;
    for k=1:2*L+1
        Zpred=Zpred+Wm(k)*Zsigmapre(:,k);
    end
    %（8）计算观测加权方差
    Pzz=0;
    for k=1:2*L+1
        Pzz=Pzz+Wc(k)*(Zsigmapre(:,k)-Zpred)*(Zsigmapre(:,k)-Zpred)';
    end
    Pzz=Pzz+R;
    
    %（9）计算预测协方差
    Pxz=zeros(L,1);
    for k=1:2*L+1
        Pxz=Pxz+Wc(k)*(Xaugsigma(:,k)-Xpred)*(Zsigmapre(:,k)-Zpred)';
    end
    
    %（10）计算kalman增益
    K=Pxz*Pzz^-1;
    
    %（11）状态更新
    Xukf(:,t)=Xpred+K*(Z(t)-Zpred);
    
    %（12）方差更新
    P0=Ppred-K*Pzz*K';
end


for i=1:N
    Err_Obs(i)=RMS(X(:,i),Z(:,i));%滤波前的误差
    Err_UKF(i)=RMS(X(:,i),Xukf(:,i));%滤波后的误差
end
mean_Obs=mean(Err_Obs);
mean_UKF=mean(Err_UKF);

%%%%%%%%%%%
%画图
figure
hold on ;box on
t=(0:1:N-1);
plot(t,X(1,:),'--b','LineWidth',1);%实际轨迹
% plot(t,Z(1,:),'-or','LineWidth',1);%观测轨迹
plot(t,Xukf(1,:),':k','LineWidth',2);%卡尔曼滤波轨迹
% plot(X(1,:),X(3,:),'-k.');
% plot(Xukf(1,:),Xukf(3,:),'-r+');
legend('真实轨迹','UKF轨迹');
xlabel('横坐标 T/s');
ylabel('纵坐标 X/m');

figure
hold on;box on;
plot(t,Err_Obs,'-');
plot(t,Err_UKF,'--');
% plot(t,Err_EKF1,'-.');
% legend('滤波前误差',num2str(mean_Observation),'基本滤波后误差','固定增益滤波后误差');
legend(sprintf('滤波前误差%.03f',mean_Obs),sprintf('UKF滤波后误差%.03f',mean_UKF));
xlabel('观测时间/s');
ylabel('误差值');

%%%%%%%%%%%%%
%子函数
% 计算欧氏距离子函数
function dist=RMS(X1,X2)
if length(X2)<=2
    dist=sqrt((X1(1)-X2(1))^2);
else
    dist=sqrt((X1(1)-X2(1))^2);
end
end

function res=gfun(Xekf,k)
    res=0.5.*Xekf + 25.*Xekf./(1 + Xekf.^2) + 8.*cos(0.3.*(k));
end

function res=hfun(X,k)
res=X^2/20;
end
%%%%%%%%%%%%%