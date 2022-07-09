clc;clear;

syms n
R_xxm=(4/5)^n - (5/4)^n + kroneckerDelta(n, 0);
syms z
f=0.36/((1-0.8*z^(-1))*(1-0.8*z))+1;
R_xxz=ztrans(R_xxm);
ff=iztrans(f);
[n,d]=numden(R_xxz);
pretty(R_xxz)
num=sym2poly(n);
den=sym2poly(d);
% sysc=tf(num,den);
[z1,p1,k1]=tf2zp(num,den);
sysc_zpk=zpk(z1,p1,k1)