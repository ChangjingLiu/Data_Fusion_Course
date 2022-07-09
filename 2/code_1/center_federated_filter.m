%%
clc
clear all;
close all;
%1.建一个匀速圆周运动的模型

r = 500;
x = -r:0.01:r;
y = sqrt(r.^2 - x.^2);   %运动的轨迹

% plot(x,[y;-y],'LineWidth',2);
% grid on;axis equal;title('质点轨迹','FontSize',16)
% xlabel('x/m','FontSize',16)
% ylabel('y/m','FontSize',16)
% xlim([-2000 2000])
% ylim([-2000 2000])
% text(750,750,'o','color','g')
% text(750,750,'雷达1','FontSize',10)
% text(-750,750,'o','color','g')
% text(-750,750,'雷达2','FontSize',10)
% text(0,-1000,'o','color','g')
% text(0,-1000,'雷达3','FontSize',10)

%%
%目标的移动
Observation_time = 0.5;

v = 10;
w = v/r;%角速度
t = 0:Observation_time:1.5*pi/w;%刚好转一圈
Q = 2;%过程噪声
x_w=sqrt(Q)*randn(1,size(t,2));
y_w=sqrt(Q)*randn(1,size(t,2));

x_traj = r * sin(w*t);%实现刚好走完一圈
y_traj = r * cos(w*t);
s_traj = [x_traj;y_traj];

x_ture_traj = r * sin(w*t)+x_w;%实现刚好走完一圈
y_ture_traj = r * cos(w*t)+y_w;
ture_traj = [x_ture_traj;y_ture_traj];


%雷达的位置
radar_1 = [750 750];
radar_2 = [-750 750];
radar_3 = [0 -1000];
radars=[radar_1;radar_2;radar_3];
radar1_noise = 3;
radar2_noise = 1;
radar3_noise = 4;
radars_noise=diag([3,1,4]);

%真实的测量信号
radar1_ture_measuremnet = sqrt((radar_1(1)-x_ture_traj).^2 + (radar_1(2)-y_ture_traj).^2 );
radar2_ture_measuremnet = sqrt((radar_2(1)-x_ture_traj).^2 + (radar_2(2)-y_ture_traj).^2 );
radar3_ture_measuremnet = sqrt((radar_3(1)-x_ture_traj).^2 + (radar_3(2)-y_ture_traj).^2 );

%加噪声的测量
radar1_noise_measuremnet = radar1_ture_measuremnet + radar1_noise * randn(1,size(radar1_ture_measuremnet,2));
radar2_noise_measuremnet = radar2_ture_measuremnet + radar2_noise * randn(1,size(radar2_ture_measuremnet,2));
radar3_noise_measuremnet = radar3_ture_measuremnet + radar3_noise * randn(1,size(radar3_ture_measuremnet,2));
radars_noise_measuremnet=[radar1_noise_measuremnet;radar2_noise_measuremnet;radar3_noise_measuremnet];


%%
%FR结构的联邦滤波器(融合重置结构)
%联邦滤波器 三个子滤波器 一个主滤波器
beta_M = 0;
N = 3;
beta_i = 1/N;%每个传感器分配的权值是一样的

Q_process_noise_system = diag([2 2]);%系统的过程噪声，题目的要求
QQ=blkdiag(Q_process_noise_system,Q_process_noise_system,Q_process_noise_system);
P_cov_system = diag([1000 1000 ]);%初始的系统协方差,随便给定的
X_init_state = [0;500];%初始目标的位置
%%%%%%%%%%%%%%%%%%%%%%%%%%%%1.信息分配与重置%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Q_process_noise_radar =  1/(beta_i) * Q_process_noise_system;

system_init_pos  = X_init_state;
object1_init_pos = X_init_state;
object2_init_pos = X_init_state;
object3_init_pos = X_init_state;
objects_init_pos=[object1_init_pos;object2_init_pos;object3_init_pos];

object1_correct_pos = X_init_state;
object2_correct_pos = X_init_state;
object3_correct_pos = X_init_state;
fusion_pos          = X_init_state;
central_fusion_pos          = X_init_state;

object1_init_cov = 1/(beta_i) * P_cov_system;
object2_init_cov = 1/(beta_i) * P_cov_system;
object3_init_cov = 1/(beta_i) * P_cov_system;
objects_init_cov = blkdiag(object1_init_cov,object2_init_cov,object3_init_cov);

%集中式卡尔曼
tic;
for i = 1:size(radar1_ture_measuremnet,2)-1 %第一个观测值扔掉
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%2.传感器时间量测更新%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %由于是匀速圆周运动，所以可以用旋转矩阵来做预测 R = [cos(theta) -sin(theta) ; sin(theta) cos(theta)]
    %由于旋转矩阵本身的方向是逆时针方向的，而本题物体的运动是瞬时间的，所以需要用到旋转矩阵的逆 = 旋转矩阵的转置
    theta = w * Observation_time * 1;
    R = [cos(theta)  -sin(theta);
        sin(theta) cos(theta)]'; %对于每个传感器都是一样的
    %预测
    F = R;
    FF=blkdiag(R,R,R);
    objects_estimate_pos(:,i+1) =  FF * objects_init_pos;%集中滤波器
    P_objects_estimate          =  FF * objects_init_cov * FF' + QQ;%集中滤波器
    
    %更新
    %因为观测并不是线性的，所以这里需要用到拓展卡尔曼滤波，对观测方程进行求导
    %观测矩阵也是不断变化的，因为是在均值处进行展开
    dec1 = sqrt((radar_1(1)-objects_estimate_pos(1,i+1)).^2 + (radar_1(2)-objects_estimate_pos(2,i+1)).^2);
    dec2 = sqrt((radar_2(1)-objects_estimate_pos(3,i+1)).^2 + (radar_2(2)-objects_estimate_pos(4,i+1)).^2);
    dec3 = sqrt((radar_3(1)-objects_estimate_pos(5,i+1)).^2 + (radar_3(2)-objects_estimate_pos(6,i+1)).^2);
    decs=[dec1;dec2;dec3];
    %     decs=
    H1 =[ -(radar_1(1)-objects_estimate_pos(1,i+1))/dec1  -(radar_1(2)-objects_estimate_pos(2,i+1))/dec1];
    H2 =[ -(radar_2(1)-objects_estimate_pos(3,i+1))/dec2  -(radar_2(2)-objects_estimate_pos(4,i+1))/dec2];
    H3 =[ -(radar_3(1)-objects_estimate_pos(5,i+1))/dec3  -(radar_3(2)-objects_estimate_pos(6,i+1))/dec3];
    HH=blkdiag(H1,H2,H3);
    KK  = P_objects_estimate * HH' * (HH * P_objects_estimate * HH' + radars_noise)^-1 ;
    objects_correct_pos(:,i+1) = objects_estimate_pos(:,i+1) + KK * ( radars_noise_measuremnet(:,i+1) - decs);%集中滤波器
    objects_init_cov = (eye(6) - KK * HH)*P_objects_estimate;%集中滤波器
    central_fusion_pos(:,i+1)=[mean(objects_correct_pos([1 3 5],i+1));mean(objects_correct_pos([2 4 6],i+1))];
    objects_init_pos = objects_correct_pos(:,i+1);
end
toc;
tic;
for i = 1:size(radar1_ture_measuremnet,2)-1 %第一个观测值扔掉
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%2.传感器时间量测更新%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %由于是匀速圆周运动，所以可以用旋转矩阵来做预测 R = [cos(theta) -sin(theta) ; sin(theta) cos(theta)]
    %由于旋转矩阵本身的方向是逆时针方向的，而本题物体的运动是瞬时间的，所以需要用到旋转矩阵的逆 = 旋转矩阵的转置
    theta = w * Observation_time * 1;
    R = [cos(theta)  -sin(theta);
        sin(theta) cos(theta)]'; %对于每个传感器都是一样的
    %%
    %传感器1
    %嵌套
    %radar1的卡尔曼子滤波器
    %预测
    F = R;
%     FF=blkdiag(R,R,R);
    object1_estimate_pos(:,i+1) =  F * object1_init_pos;
%     objects_estimate_pos(:,i+1) =  FF * objects_init_pos;%集中滤波器
    P_object1_estimate          =  F * object1_init_cov * F' + Q_process_noise_radar;
%     P_objects_estimate          =  FF * objects_init_cov * FF' + QQ;%集中滤波器
    
    %更新
    %因为观测并不是线性的，所以这里需要用到拓展卡尔曼滤波，对观测方程进行求导
    %观测矩阵也是不断变化的，因为是在均值处进行展开
    dec = sqrt((radar_1(1)-object1_estimate_pos(1,i+1)).^2 + (radar_1(2)-object1_estimate_pos(2,i+1)).^2);
%     dec1 = sqrt((radar_1(1)-objects_estimate_pos(1,i+1)).^2 + (radar_1(2)-objects_estimate_pos(2,i+1)).^2);
%     dec2 = sqrt((radar_2(1)-objects_estimate_pos(3,i+1)).^2 + (radar_2(2)-objects_estimate_pos(4,i+1)).^2);
%     dec3 = sqrt((radar_3(1)-objects_estimate_pos(5,i+1)).^2 + (radar_3(2)-objects_estimate_pos(6,i+1)).^2);
%     decs=[dec1;dec2;dec3];
    %     decs=
    H = [ -(radar_1(1)-object1_estimate_pos(1,i+1))/dec  -(radar_1(2)-object1_estimate_pos(2,i+1))/dec];
%     H1 =[ -(radar_1(1)-objects_estimate_pos(1,i+1))/dec1  -(radar_1(2)-objects_estimate_pos(2,i+1))/dec1];
%     H2 =[ -(radar_2(1)-objects_estimate_pos(3,i+1))/dec2  -(radar_2(2)-objects_estimate_pos(4,i+1))/dec2];
%     H3 =[ -(radar_3(1)-objects_estimate_pos(5,i+1))/dec3  -(radar_3(2)-objects_estimate_pos(6,i+1))/dec3];
%     HH=blkdiag(H1,H2,H3);
    K  = P_object1_estimate * H' * (H * P_object1_estimate * H' + radar1_noise)^-1 ;
%     KK  = P_objects_estimate * HH' * inv(HH * P_objects_estimate * HH' + radars_noise) ;
    object1_correct_pos(:,i+1) = object1_estimate_pos(:,i+1) + K * ( radar1_noise_measuremnet(i+1) - sqrt((radar_1(1)-object1_estimate_pos(1,i+1)).^2 + (radar_1(2)-object1_estimate_pos(2,i+1)).^2));
%     objects_correct_pos(:,i+1) = objects_estimate_pos(:,i+1) + KK * ( radars_noise_measuremnet(:,i+1) - decs);%集中滤波器
    inform_object1_correct = inv((eye(2) - K * H)*P_object1_estimate);
%     objects_init_cov = (eye(6) - KK * HH)*P_objects_estimate;%集中滤波器
    
    %radar2的卡尔曼子滤波器
    
    object2_estimate_pos(:,i+1) =  F * object2_init_pos;
    P_object2_estimate          =  F *  object2_init_cov * F' + Q_process_noise_radar;
    
    %更新
    
    dec = sqrt((radar_2(1)-object2_estimate_pos(1,i+1)).^2 + (radar_2(2)-object2_estimate_pos(2,i+1)).^2);
    H = [ -(radar_2(1)-object2_estimate_pos(1,i+1))/dec -(radar_2(2)-object2_estimate_pos(2,i+1))/dec];
    
    
    K  = P_object2_estimate * H' * (H * P_object2_estimate * H' + radar2_noise)^-1 ;
    object2_correct_pos(:,i+1) = object2_estimate_pos(:,i+1) + K * ( radar2_noise_measuremnet(i+1) - sqrt((radar_2(1)-object2_estimate_pos(1,i+1)).^2 + (radar_2(2)-object2_estimate_pos(2,i+1)).^2));
    inform_object2_correct = inv((eye(2) - K * H)*P_object2_estimate);
    
    %%
    %radar3的卡尔曼子滤波器
    %object3_init_pos
    object3_estimate_pos(:,i+1) =  F * object3_init_pos;
    P_object3_estimate          =  F *  object3_init_cov * F' + Q_process_noise_radar;
    
    %更新
    
    dec = sqrt((radar_3(1)-object3_estimate_pos(1,i+1)).^2 + (radar_3(2)-object3_estimate_pos(2,i+1)).^2);
    H = [ -(radar_3(1)-object3_estimate_pos(1,i+1))/dec -(radar_3(2)-object3_estimate_pos(2,i+1))/dec];
    
    
    
    K  = P_object3_estimate * H' * (H * P_object3_estimate * H' + radar3_noise)^-1 ;
    object3_correct_pos(:,i+1) = object3_estimate_pos(:,i+1) + K * ( radar3_noise_measuremnet(i+1) - sqrt(     (radar_3(1)-object3_estimate_pos(1,i+1)).^2 + (radar_3(2)-object3_estimate_pos(2,i+1)).^2)  );
    inform_object3_correct = inv((eye(2) - K * H)*P_object3_estimate);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%3.主滤波器时间更新%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %主滤波器 有3种模式，NR,ZR,FS
    
    system_estimate_pos = F * system_init_pos;
    system_estimate_cov = F * P_cov_system * F';
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%4.最后的融合处理%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fusion_cov = inv( inform_object1_correct + inform_object2_correct + inform_object3_correct + inv(system_estimate_cov));
    fusion_pos(:,i+1) = fusion_cov* (inform_object1_correct *object1_correct_pos(:,i+1) + inform_object2_correct * object2_correct_pos(:,i+1) + inform_object3_correct *object3_correct_pos(:,i+1) + inv(system_estimate_cov) * system_estimate_pos );
    
    
    object1_init_pos = fusion_pos(:,i+1);
    object1_init_cov = 1/(beta_i) * fusion_cov;
    object2_init_pos = fusion_pos(:,i+1);
    object2_init_cov = 1/(beta_i) * fusion_cov;
    object3_init_pos = fusion_pos(:,i+1);
    object3_init_cov = 1/(beta_i) * fusion_cov;
    
    system_init_pos = fusion_pos(:,i+1);
    P_cov_system = fusion_cov ;
    
end
toc;

% plot(x,[y;-y],'LineWidth',2);
grid on;axis equal;title('质点轨迹','FontSize',16)
xlabel('x/m','FontSize',16)
ylabel('y/m','FontSize',16)
xlim([-1500 1500])
ylim([-1500 1500])
text(750,750,'o','color','g')
text(750,750,'传感器1','FontSize',10)
text(-750,750,'o','color','g')
text(-750,750,'传感器2','FontSize',10)
text(0,-1000,'o','color','g')
text(0,-1000,'传感器3','FontSize',10)
hold on
plot(fusion_pos(1,:),fusion_pos(2,:));
plot(central_fusion_pos(1,:),central_fusion_pos(2,:));
legend("联邦卡尔曼滤波","集中式滤波");
% comet(fusion_pos(1,:),fusion_pos(2,:))
% comet(central_fusion_pos(1,:),central_fusion_pos(2,:))

for i=1:size(radar1_ture_measuremnet,2)-1
    %     Err_S(i)=Dist(ture_traj(:,i),s_traj(:,i));%滤波前的误差
    Err_FF(i)=Dist(fusion_pos(:,i),ture_traj(:,i));%滤波前的误差
    Err_CFF(i)=Dist(central_fusion_pos(:,i),ture_traj(:,i));%滤波前的误差
    %     Err_UKF(i)=RMS(X(:,i),Xukf(:,i));%滤波后的误差
end
% mean_S=mean(Err_S);
mean_FF=mean(Err_FF);
mean_CFF=mean(Err_CFF);
% mean_UKF=mean(Err_UKF);
% mean_PF=mean(Err_PF);
t=1:size(radar1_ture_measuremnet,2)-1;
figure
hold on;box on;
% plot(t,Err_Obs,'-');
% plot(t,Err_UKF,'--');
% plot(t,Err_S,'-.');
plot(t,Err_FF,'-');
plot(t,Err_CFF,'-.');
% legend('滤波前误差',num2str(mean_Observation),'基本滤波后误差','固定增益滤波后误差');
legend(sprintf('联邦滤波后误差%.03f',mean_FF),sprintf('集中式滤波后误差%.03f',mean_CFF));
xlabel('观测时间/s');
ylabel('误差值');

function d=Dist(X1,X2)
if length(X2)<=2
    d=sqrt((X1(1,1)-X2(1,1))^2+(X1(2,1)-X2(2,1))^2);
else
    %     d=sqrt((X1(1)-X2(1))^2+(X1(3)-X2(3))^2);
end
end