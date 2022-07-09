function Xukf=UKF(Xukf,Z,N,Q,R)

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
end

function res=gfun(Xekf,k)
res=0.5.*Xekf + 25.*Xekf./(1 + Xekf.^2) + 8.*cos(0.4.*(k));
end

function res=hfun(X,k)
res=X^2/20;
end