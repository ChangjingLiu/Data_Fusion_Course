function Xkf=kalman_gainfix(Xkf,u,Z,H,P0,Q,R,phi,Tau,N)
%% 常值增益kalman滤波
K=[0.5;0.1];
for i=2:N
    Xn=phi*Xkf(:,i-1)+Tau*u;%预测
    P1=phi*P0*phi'+Q;%预测误差协方差
%     K=P1*H'*(H*P1*H'+R)^(-1);%增益
    Xkf(:,i)=Xn+K*(Z(:,i)-H*Xn);%状态更新
    P0=(eye(2)-K*H)*P1;             %滤波误差协方差更新
end