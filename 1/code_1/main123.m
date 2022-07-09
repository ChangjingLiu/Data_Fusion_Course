%%main
clear;clc;
%%状态值x误差满足均值为5，方差为0.01的正态分布
%%仪器1测量误差满足均值为0，方差为0.1的正态分布
%%仪器2测量误差满足均值为0，方差为0.4的正态分布
x=5; %状态值x均值
var0=0.1; %状态方差var0
var1=0.1; %测量方差var1
var2=0.4; %测量方差var2

r=20; %观测次数，增大观测次数
s = rng;
z1 = x+sqrt(var0)*randn(1,r)+sqrt(var1)*randn(1,r); %观测值z1
z2 = x+sqrt(var0)*randn(1,r)+sqrt(var2)*randn(1,r); %观测值z1
z=[z1';z2'];%观测矩阵z
h=ones(1,2*r)';%测量矩阵h

v1=ones(1,r)*var1;
v2=ones(1,r)*var2;
R=diag([v1,v2]); %观测方差矩阵R

[X_hat1,MSE1]=LSM(h,z,R);%最小二乘法估计
[X_hat2,MSE2]=WLSM(h,z,R,R^(-1));%加权最小二乘法估计 R^(-1)为最优权重
[X_hat3,MSE3]=MVE(h,z,x,var0,R);% 线性最小方差估计
