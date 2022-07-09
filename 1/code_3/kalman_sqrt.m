function Xkf=kalman_sqrt(Xkf,u,Z,H,P0,Q,R,Phi,Tau,N)
sQ = mychol(Q); sR = mychol(R); Delta0 = mychol(P0);
%% 平方根kalman滤波
    for i=2:N
        %状态更新
        Xn=Phi*Xkf(:,i-1)+Tau*u;%预测
        
        %预测协方差更新
        [~, Delta] = myqr([Phi*Delta0, Tau*sQ]');  %[Phi*Delta0, Tau*sQ]'系统驱动噪声
        Delta = Delta';%delta_k+1/k=U^T上三角阵,这里已经完成了计算
    
        [~, rho] = myqr([H*Delta, sR]'); 
        rho = rho';%相当于
        
        %测量更新
        K=Delta*Delta'*H'*(rho*rho')^(-1);
        Xkf(:,i)=Xn+K*(Z(:,i)-H*Xn);%状态更新
        Delta0 = Delta*(eye(length(Delta0))-Delta'*H'*(rho*rho'+sR*rho')^-1*H*Delta);
    end
end

function [Phi, Tau, H, Q, R, P0] = rndmodel(n, m, l)  % 随机系统模型
    Phi = randn(n);  Tau = randn(n,l);  H = randn(m,n);
    Q = diag(randn(l,1))^2;  R = diag(randn(m,1))^2;
    P0 = randn(n); P0 = P0'*P0;
end

function Delta1 = SRKF(Delta0, Phi, Tau, sQ, H, sR)  % 平方根滤波(核心时不断更新协方差的平方根)
    %Delta0：协方差P的平方根
    %phi：
    %tau：
    %sq：Q平方根
    %H：H测量矩阵
    %sr：R平方根
    %%预测
    [q, Delta] = myqr([Phi*Delta0, Tau*sQ]');  %[Phi*Delta0, Tau*sQ]'系统驱动噪声
    Delta = Delta';%delta_k+1/k=U^T上三角阵,这里已经完成了计算
    
    [q, rho] = myqr([H*Delta, sR]'); 
    rho = rho';%相当于
    Delta1 = Delta*(eye(length(Delta0))-Delta'*H'*(rho*rho'+sR*rho')^-1*H*Delta);
end

function A = mychol(P)  % 乔莱斯基分解，P=A*A', A为上三角阵
    n = length(P);  A = zeros(n);
    for j=n:-1:1
       A(j,j) = sqrt(P(j,j)-A(j,j+1:n)*A(j,j+1:n)');
       for i=(j-1):-1:1
           A(i,j) = (P(i,j)-A(i,j+1:n)*A(j,j+1:n)')/A(j,j);
       end
    end
end

function [Q, R] = myqr(A)  % QR分解，A=Q*R, 其中Q'*Q=I，R为上三角阵
    [m, n] = size(A);
    if n>m,  error('n must not less than m.'); end
    R = zeros(n);
    for i=1:n
       R(i,i) = sqrt(A(:,i)'*A(:,i));
       A(:,i) = A(:,i)/R(i,i);
       j = i+1:n;
       R(i,j) = A(:,i)'*A(:,j);
       A(:,j) = A(:,j)-A(:,i)*R(i,j);
    end
    Q = A;
    
end
