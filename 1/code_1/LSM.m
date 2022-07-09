function [X_hat,MSE]=LSM(H,Z,R)
 X_hat=(H'*H)^(-1)*H'*Z;
 MSE=(H'*H)^(-1)*H'*R*H*(H'*H)^(-1);%均方误差
end

