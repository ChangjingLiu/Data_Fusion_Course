clear;clc;close all;
% 观测点数
N = 500;
x = linspace(-2*pi, 2*pi,N );
% 信号
y = 3 * sin(x)+2*cos(x.^2);
% 噪声（方差1.25）
v1 = 0.05*randn(N , 1).*y';
v2 = 0.05*randn(N , 1);
v=v1+v2;
z=y'+v;
d=y';
P=x;
T=z';

%%% 画图
% plot(x,y);
% hold on;
% plot(x,T);
% axis([-2*pi 2*pi -6 6]);
% legend('y-x曲线','z-x曲线');
% xlabel('横坐标 x');
% ylabel('纵坐标 y');
% 
% figure;
% plot(x,v');
% axis([-2*pi 2*pi -0.7 0.7]);
% legend('v-x曲线');
% xlabel('横坐标 x');
% ylabel('纵坐标 y');

% 数据集设置
[a,b]=dividerand([P;T],0.8,0.2); %训练集+验证集 80%，测试集20%
trainp=a(1,:);
traint=a(2,:);
testp=b(1,:);
testt=b(2,:);

%BP网络创建

net=newff(P,T,[50]);


% 指定训练参数
net.trainFcn='trainlm';

%BP网络参数的设置
net.trainParam
net.trainParam.epochs=300;%训练次数设置
net.trainParam.goal=1e-7;%训练目标设置
net.trainParam.lr=0.001;%学习率设置,应设置为较少值，太大虽然会在开始加快收敛速度，但临近最佳点时，会产生动荡，而致使无法收敛
net.trainParam.mc=0.9;%动量因子的设置，默认为0.9
net.trainParam.show=25;%显示的间隔次数
net.divideParam.trainRatio=8/8; %训练集 60%
net.divideParam.valRatio=0;%验证集 20%
net.divideParam.testRatio=0;%验证集 20%


%RBF网络创建与训练
% net_rbf=newrb(trainp,traint,1);

%RBF参数设置
% net_rbf.trainParam
% net_rbf.trainParam.epochs=10000;%训练次数设置


%训练网络
start1=tic;
% [net,tr]=train(net,trainp,traint);
% toc;
time1=toc(start1);

start2=tic;
%RBF网络创建及训练
net_rbf=newrb(trainp,traint,1e-7,0.8,100,25);
[net_rbf,tr]=train(net_rbf,trainp,traint);
time2=toc(start2);

% [net_rbf,tr_rbf]=train(net_rbf,trainp,traint);
% 测试结果
[output]=sim(net,testp);%训练的数据，根据BP得到的结果
[output_rbf] = sim(net_rbf,testp);
test_mse = mse(output-testt);
test_mse_rbf = mse(output_rbf-testt);

figure;

plot(x,d);
axis([-2*pi 2*pi -6 6]);
hold on;
plot(trainp,traint);
plot(testp,output,"o");
plot(testp,output_rbf,"x");
title("BF网络与RBF的对比图");
% legend('理论输出','实际输出','BP预测点','RBF预测点');
legend('理论输出','实际输出',sprintf('BP预测点%.03f 用时%.03f',test_mse,time1),sprintf('RBF预测点%.03f 用时%.03f',test_mse_rbf,time2));
xlabel('横坐标 X');
ylabel('纵坐标 Y');

% figure;
% plot(x,d);
% hold on;
% plot(trainp,traint);
% plot(testp,output_rbf,"o");
% % plot(testp,testt,"x");
% title("RBF");
% legend('理论输出','实际输出','预测点');
% xlabel('横坐标 T/s');
% ylabel('纵坐标 X/m');

% %%%%%%%%%%%
% %画图
% figure
% hold on ;box on
% t=(0:1:N-1);
% plot(t,X(1,:),'--b','LineWidth',1);%实际轨迹
% % plot(t,Z(1,:),'-or','LineWidth',1);%观测轨迹
% plot(t,Xukf(1,:),':k','LineWidth',2);%卡尔曼滤波轨迹
% % plot(X(1,:),X(3,:),'-k.');
% % plot(Xukf(1,:),Xukf(3,:),'-r+');
% legend('真实轨迹','UKF轨迹');
% xlabel('横坐标 T/s');
% ylabel('纵坐标 X/m');
% 
% figure
% hold on;box on;
% plot(t,Err_Obs,'-');
% plot(t,Err_UKF,'--');
% % plot(t,Err_EKF1,'-.');
% % legend('滤波前误差',num2str(mean_Observation),'基本滤波后误差','固定增益滤波后误差');
% legend(sprintf('滤波前误差%.03f',mean_Obs),sprintf('UKF滤波后误差%.03f',mean_UKF));
% xlabel('观测时间/s');
% ylabel('误差值');
% 
% %%%%%%%%%%%%%
% %子函数
% % 计算欧氏距离子函数
% function dist=RMS(X1,X2)
% if length(X2)<=2
%     dist=sqrt((X1(1)-X2(1))^2);
% else
%     dist=sqrt((X1(1)-X2(1))^2);
% end
% end
