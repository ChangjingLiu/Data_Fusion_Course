function [X_hat,P]=MVE(H,Z,mx,Cx,Cz)  
    X_hat=mx+Cx*H'*(H*Cx*H'+Cz)^(-1)*(Z-H*mx);
    P=(Cx^(-1)+H'*Cz^(-1)*H)^(-1);
end

