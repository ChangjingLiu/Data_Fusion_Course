function Xkf=kalman_restrain_K(Xkf,u,Z,H,P0,Q,R,phi,Tau,N)
    %% 限制k的kalman滤波
    K=[0;0];
    for i=2:N
        Xn=phi*Xkf(:,i-1)+Tau*u;%预测
        P1=phi*P0*phi'+Q;%预测误差协方差
        e=Z(:,i)-H*Xn;  %新息
        E=H*P0*H'+R;    %量测估计误差
        r=3;
        if(e'*e>r*trace(E))
           disp("发散");
           %重置k
%            K=[0.5;0.1];
            if(i==2)
                K=P1*H'*(H*P1*H'+R)^(-1);%增益
            end
        else
            K=P1*H'*(H*P1*H'+R)^(-1);%增益
        end
        Xkf(:,i)=Xn+K*(Z(:,i)-H*Xn);%状态更新
        P0=(eye(2)-K*H)*P1;             %滤波误差协方差更新
    end
end