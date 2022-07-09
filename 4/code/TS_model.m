clear;clc;
close all;
%数据点个数51 
numpts=500; 
x1=linspace(-2*pi,2*pi,numpts); 
d=3 * sin(x1)+2*cos(x1.^2); 
% 噪声（方差1.25）
v1 = 0.05*randn(numpts , 1).*d';
v2 = sqrt(0.5)*randn(numpts , 1);
v=v1+v2;%噪声信号
y=d+v';
data=[x1' y'];     %整个数据集 
[train,test]=dividerand(data',0.8,0.2);
% trndata=data(1:2:numpts,:);    %训练数据集 
% chkdata=data(2:2:numpts,:);    %测试数据集 
trndata=train';
chkdata=test';
%训练数据和检验数据的分布曲线 
% plot(trndata(:,1),trndata(:,2),'o',chkdata(:,1),chkdata(:,2),'x') 
 
%建立T_S模糊模型 
%采用genfis1()函数直接由训练数据生成模糊推理系统 
nummfs=20;           %隶属度函数个数 
mftype='gbellmf';    %隶属度函数类型 gbellmf钟形隶属度函数
fismat=genfis1(trndata,nummfs,mftype); 
% showrule(fismat)
% in = getTunableSettings(fismat)

% 建立 Mamdani模糊系统
% opt1 = genfisOptions('FCMClustering' , 'FISType' , 'mamdani' );
% opt1.NumClusters = 30;
% fis = genfis (x1', y', opt1);
% showrule(fis)
dataRange = [min(data)' max(data)'];
fisin = mamfis;
fisin = addInput(fisin,[-2*pi,2*pi], 'Name' ,'in', 'NumMFs' ,32,'mftype','gaussmf');
fisin = addOutput(fisin,[-6.534905896158901,6.092046113952383], 'Name' ,'out', 'NumMFs' ,64,'mftype','gaussmf');
figure
plotfis(fisin)

% %绘制模糊推理系统的初始隶属度函数 
% [x,mf]=plotmf(fismat,'input',1); 
% figure 
% plot(x,mf); 
% title('initial menbership functions') 
 
%使用函数anfis()进行神经模糊建摸 
numepochs=100;   %训练次数40 
opt=anfisOptions("InitialFIS",fismat,"EpochNumber",numepochs,"ErrorGoal",0.005,"InitialStepSize",0.01,"StepSizeDecreaseRate",0.9,"ValidationData",chkdata);
[fismat1,truerr,ss,fismat2,chkerr]=anfis(trndata,opt);
%计算训练后神经模糊系统的输出与训练数据的均方根误差 
trnout=evalfis(trndata(:,1),fismat1); 
trnrmse=norm(trnout-trndata(:,2))/sqrt(length(trnout)); 


%使用函数tunefis()调整Mamdani模糊推理模型
% 1学习规则
options = tunefisOptions( 'Method' , 'particleswarm' , ... 
    'OptimizationType' , 'learning' , ... 
    'NumMaxRules' ,64);
options.MethodOptions.MaxIterations = 50;
options.UseParallel=true;
% [in,out,rule] = getTunableSettings(fis);
fisout1=tunefis(fisin,[],trndata(:,1),trndata(:,2),options);
figure
plotfis(fisout1)

%2 调整所有参数
[in,out,rule] = getTunableSettings(fisout1);
options.OptimizationType = 'tuning';
options.Method = 'patternsearch';
options.MethodOptions.MaxIterations = 60;
options.MethodOptions.UseCompletePoll = true;
fisout = tunefis(fisout1,[in;out;rule],trndata(:,1),trndata(:,2),options); 
figure
plotfis(fisout)


%绘制训练过程中均方根误差的变化情况 
epoch=1:numepochs; 
figure 
plot(epoch,truerr,'o',epoch,chkerr,'x') 
hold on 
plot(epoch,[truerr,chkerr]); 
hold off 
title('误差曲线');
legend("训练集误差","验证集误差");
 
% %绘制训练过程中的步长的变化的情况 
% figure 
% plot(epoch,ss,'-',epoch,ss,'x'); 
% title('步长变化曲线') 
 
% %绘制训练后模糊推理系统的输入隶属度函数曲线 
% [x,mf]=plotmf(fismat1,'input',1); 
% figure 
% plot(x,mf) 
% title('训练后的输入隶属度函数'); 

% % %绘制训练后模糊推理系统的输入隶属度函数曲线 
% % % figure 
% plotmf(fismat1,'output',1); 
% title('训练后的输出隶属度函数'); 
 
%绘制神经模糊推理系统的输出曲线 
pre_y=evalfis(x1,fismat1); %系统辨识
pre_y_1=evalfis(x1,fisout1); %系统辨识
mse1=norm(y-d)/sqrt(length(d)); 
mse=norm(pre_y-d')/sqrt(length(d')); 
mse_mamfis=norm(pre_y_1-d')/sqrt(length(d')); 

figure 
plot(x1,d,'-r');
hold on;
plot(x1,y,'-b');
hold on;
plot(x1,pre_y_1,'--',"LineWidth",2)
title('系统辨识曲线');
legend("理想系统",sprintf('辨识前的系统：误差%.03f',mse1),sprintf('模糊集合辨识后的系统：误差%.03f',mse_mamfis));

%总曲线
figure 
plot(x1,d,'-r');
hold on;
plot(x1,y,'-b');
hold on;
plot(x1,pre_y,'--k',"LineWidth",2)
hold on;
plot(x1,pre_y_1,'--',"LineWidth",2)
title('系统辨识曲线');
legend("理想系统",sprintf('辨识前的系统：误差%.03f',mse1),sprintf('T-S模型辨识后的系统：误差%.03f',mse),sprintf('模糊集合辨识后的系统：误差%.03f',mse_mamfis));