
function Hopt=IIR_wf(x,d,n)
%%% 维纳滤波
Rxx=xcorr(x);   %自相关
Gxx=fft(Rxx,n); %自相关功率谱密度
Rxy=xcorr(x,d); %互相关
Gxs=fft(Rxy,n); %互相关功率谱密度
Hopt=Gxs./Gxx;     %维纳霍夫方程 满足最小均方误差的滤波因子H
end

% Ps=fft(y);      %原始信号的功率谱密度
% S=H.*Ps;        %经过满足最小均方误差滤波
% ss=ifft(S);     %信号还原
