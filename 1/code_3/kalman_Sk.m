function Xkf=kalman_Sk(Xkf,u,Z,H,P0,Q,R,phi,Tau,N)
    %% 扩大P(k|k-1)的kalman滤波(Sk法)
    for i=2:N
        Xn=phi*Xkf(:,i-1)+Tau*u;%预测
        e=Z(:,i)-H*Xn;  %新息
        E=H*P0*H'+R;    %量测估计误差
        r=3;
        if(e'*e>r*trace(E))
           disp("发散");
           Sk=trace(e*e'-H*Q*H'-R)/trace(H*phi*P0*phi'*H');
           P1=Sk*phi*P0*phi'+Q;%预测误差协方差
        else
            P1=phi*P0*phi'+Q;%预测误差协方差
        end
        K=P1*H'*(H*P1*H'+R)^(-1);%增益
        Xkf(:,i)=Xn+K*(Z(:,i)-H*Xn);%状态更新
        P0=(eye(2)-K*H)*P1;             %滤波误差协方差更新
    end
end