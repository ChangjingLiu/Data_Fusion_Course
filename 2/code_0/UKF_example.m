%%%
% UKF在目标跟踪中的应用
%%%
clear;clc; close all;
N = 100;
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
L=1;%4个变量
alpha=1;
kalpha=0;
belta=2;
ramda=3-L;
for j=1:2*L+1
    Wm(j)=1/(2*(L+ramda));
    Wc(j)=1/(2*(L+ramda));
end
Wm(1)=ramda/(L+ramda);%权值Wm的初值需要另外定
Wc(1)=ramda/(L+ramda)+1-alpha^2+belta;%权值Wc的初值需要另外定
%%%%%%%%%%%%%%%%
Xukf=zeros(L,N);
Xukf(:,1)=X(:,1);%把X真值的初次数据赋给Xukf
P0=eye(L);%协方差阵初始化
for t=2:N
    xestimate=Xukf(:,t-1);%获取上一步的UKF点
    P=P0;%协方差阵
    %第一步：获得一组Sigma点集
    cho=(chol(P*(L+ramda)))';
    for k=1:L
        xgamaP1(:,k)=xestimate+cho(:,k);
        xgamaP2(:,k)=xestimate-cho(:,k);
    end
    Xsigma=[xestimate,xgamaP1,xgamaP2];%xestimate是上一步的点，相当于均值点
    %第二步：对Sigma点集进行一步预测
    Xsigmapre=gfun(Xsigma,t);%F是状态转移矩阵
    %第三步：利用第二步的结果计算均值和协方差
    Xpred=zeros(L,1);
    for k=1:2*L+1
        Xpred=Xpred+Wm(k)*Xsigmapre(:,k);%均值
    end
    Ppred=zeros(L,L);
    for k=1:2*L+1
        Ppred=Ppred+Wc(k)*(Xsigmapre(:,k)-Xpred)*(Xsigmapre(:,k)-Xpred)';%协方差矩阵
    end
    Ppred=Ppred+Q;
    %第四步：根据预测值，再一次使用UT变换，得到新的sigma点集
    chor=(chol((L+ramda)*Ppred))';
    for k=1:L
        XaugsigmaP1(:,k)=Xpred+chor(:,k);
        XaugsigmaP2(:,k)=Xpred-chor(:,k);
    end
    Xaugsigma=[Xpred XaugsigmaP1 XaugsigmaP2];
    %第五步：观测预测
    for k=1:2*L+1
        Zsigmapre(1,k)=hfun(Xaugsigma(:,k));
    end
    %第六步：计算观测预测均值和协方差
    Zpred=0;
    for k=1:2*L+1
        Zpred=Zpred+Wm(k)*Zsigmapre(1,k);
    end
    Pzz=0;
    for k=1:2*L+1
        Pzz=Pzz+Wc(k)*(Zsigmapre(1,k)-Zpred)*(Zsigmapre(1,k)-Zpred)';
    end
    Pzz=Pzz+R;
    
    Pxz=zeros(L,1);
    for k=1:2*L+1
        Pxz=Pxz+Wc(k)*(Xaugsigma(:,k)-Xpred)*(Zsigmapre(1,k)-Zpred)';
    end
    %第七步：计算kalman增益
    K=Pxz*(Pzz)^(-1);
    %第八步：状态和方差更新
    xestimate=Xpred+K*(Z(:,t)-Zpred);
    P=Ppred-K*Pzz*K';
    P0=P;
    Xukf(:,t)=xestimate;
end

%误差分析
for i=1:N
    Err_KalmanFilter(i)=RMS(X(:,i),Xukf(:,i));
end
%%%%%%%%%%%
%画图
figure
hold on ;box on
plot(Z,'o');
plot(X,'-k.');
plot(Xukf,'-r+');
legend('观测轨迹','真实轨迹','UKF轨迹')
figure
hold on ; box on
plot(Err_KalmanFilter,'-ks','MarkerFace','r')

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
res=0.5.*Xekf + 25.*Xekf./(1 + Xekf.^2) + 8.*cos(0.5.*(k));
end

function res=hfun(X,k)
res=X^2/20;
end
%%%%%%%%%%%%%