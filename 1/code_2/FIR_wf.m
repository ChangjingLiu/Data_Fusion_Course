function Wopt=FIR_wf(x,d,N)
% 观测信号自相关
[C, lags] = xcorr(x, N, 'biased');
% 自相关矩阵R_xx，N 阶滤波器
R_xx = toeplitz( C(N + 1 : end) );
% x,d 互相关函数R_xd
R_xd = xcorr(d, x, N, 'biased');
R_xd = R_xd(N + 1 : end);
% 维纳-霍夫方程
Wopt = (R_xx)^(-1) * R_xd';
end