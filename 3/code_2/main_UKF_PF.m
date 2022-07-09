clear;clc; close all;
%% initialize the variables    
N = 100;  % 共进行75次 
N=N-1;
M = 100; % 粒子数，越大效果越好，计算量也越大  
x = 0.1; % initial actual state     
Q = 1;w=sqrt(Q)*randn(1,N);
R = 1;v=sqrt(R)*randn(1,N);  
  

X=zeros(1,N);
s=zeros(1,N);
Xpf=zeros(1,N);
Xukf=zeros(1,N);
X(1,1)=0.1;
s(1,1)=0.1;
Xpf(1,1)=X(1,1);
Xukf(1,1)=X(1,1);
Z=zeros(1,N);
Z(1)=X(1,1)^2/20+v(1);

for k = 2 : N+1
    % 模拟系统
    s(:,k) = gfun(s(:,k-1),k-1);
    X(:,k) = gfun(X(:,k-1),k-1)+ sqrt(Q)*randn;
    Z(:,k) =hfun(X(:,k),k)+ sqrt(R)*randn;
end

%粒子滤波pf
tic;
Xpf=PF(Xpf,Z,Q,R,M,N);  
toc;
% tic;
% Xukf=UKF(Xukf,Z,N+1,Q,R);
% toc;

for i=1:N+1
    Err_Obs(i)=RMS(X(:,i),Z(:,i));%滤波前的误差
%     Err_UKF(i)=RMS(X(:,i),Xukf(:,i));%滤波后的误差
    Err_PF(i)=RMS(X(:,i),Xpf(:,i));%滤波后的误差
end
mean_Obs=mean(Err_Obs);
% mean_UKF=mean(Err_UKF);
mean_PF=mean(Err_PF);

%%%%%%%%%%%
%画图
figure
hold on ;box on
t=(0:1:N);
plot(t,X(1,:),'-r','LineWidth',1);%实际轨迹
% plot(t,Xukf(1,:),'-.k','LineWidth',1);%卡尔曼滤波轨迹
plot(t,Xpf(1,:),'-.ob','LineWidth',1);%观测轨迹
% plot(X(1,:),X(3,:),'-k.');
legend('真实轨迹','PF轨迹');
xlabel('横坐标 T/s');
ylabel('纵坐标 X/m');

figure
hold on;box on;
plot(t,Err_Obs,'-');
% plot(t,Err_UKF,'--');
plot(t,Err_PF,'-.');
% legend('滤波前误差',num2str(mean_Observation),'基本滤波后误差','固定增益滤波后误差');
legend(sprintf('滤波前误差%.03f',mean_Obs),sprintf('PF滤波后误差%.03f',mean_PF));
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
function res=gfun(Xekf,t)
res= 0.5*Xekf + 25*Xekf/(1 + Xekf^2) + 8*cos(0.4*(t));
end

function res=hfun(X,k)
res=X^2/20;
end

