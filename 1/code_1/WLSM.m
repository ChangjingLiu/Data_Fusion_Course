function [X_hat,MSE]=WLSM(H,Z,R,W)
    X_hat=(H'*W*H)^(-1)*H'*W*Z;
    MSE=(H'*W*H)^(-1)*H'*W*R*W*H*(H'*W*H)^(-1);%均方误差
end

