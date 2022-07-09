function Xpf=PF(Xpf,Z,Q,R,M,N)
%粒子滤波初始化
V = 2; %初始分布的方差
x_P = []; % 粒子
% 用一个高斯分布随机的产生初始的粒子
for i = 1:M
    x_P(i) = Xpf(:,1) + sqrt(V) * randn;
end

%粒子滤波
for t = 1:N
    for i = 1:M
        %从先验p(x(k)|x(k-1))中采样
        x_P_update(:,i) = gfun(x_P(:,i),t) + sqrt(Q)*randn;
        %计算采样粒子的值，为后面根据似然去计算权重做铺垫
        z_update(:,i) = x_P_update(:,i)^2/20;
        %对每个粒子计算其权重，这里假设量测噪声是高斯分布。所以 w = p(y|x)对应下面的计算公式
        P_w(:,i) = (1/sqrt(2*pi*R)) * exp(-(Z(:,t+1) - z_update(:,i))^2/(2*R));
    end
    % 归一化.
    P_w = P_w./sum(P_w);
    
    %% Resampling这里没有用博客里之前说的histc函数，不过目的和效果是一样的
    for i = 1 : M
        x_P(:,i) = x_P_update(find(rand <= cumsum(P_w),1));   % 粒子权重大的将多得到后代
    end                                                     % find( ,1) 返回第一个 符合前面条件的数的 下标
    
    %状态估计，重采样以后，每个粒子的权重都变成了1/N
    Xpf(:,t+1)=mean(x_P);
    
    
end
end

function res=gfun(Xekf,t)
res= 0.5.*Xekf + 25.*Xekf./(1 + Xekf.^2) + 8.*cos(0.4.*(t));
end

function res=hfun(X,k)
res=X^2/20;
end