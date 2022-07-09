function Xkf=kalman_adaptive(Xkf,u,Z,H,P0,Q,R,phi,Tau,N)
    %% 自适应kalman滤波
    K=[1;0];
    lambda=0.95;%遗忘因子
    for i=2:N
        Xn=phi*Xkf(:,i-1)+Tau*u;    %预测
        P1=phi*P0*phi'+Q;           %预测误差协方差
        dk=(1-lambda)*(1-lambda^(i));
        vk=Z(:,i)-H*Xn;
        R=(1-dk)*R+dk*((eye(1)-H*K)*vk*vk'*(eye(1)-H*K)'+H*P0*H');     %观测误差R基于dk调节      
        K=P1*H'*(H*P1*H'+R)^(-1);   %增益
        Xkf(:,i)=Xn+K*vk;%状态更新
        P0=(eye(2)-K*H)*P1;         %滤波误差协方差更新
    end
end