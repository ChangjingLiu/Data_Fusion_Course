P=[0.1 0.7 0.8 0.8 1.0 0.3 0.0 -0.3 -0.5 -1.5;1.2 1.8 1.6 0.6 0.8 0.5 0.2 0.8 -1.5 -1.3];
T=[1 1 1 0 0 1 1 1 0 0;0 0 0 0 0 1 1 1 1 1];
[R,Q]=size(P);
[S,Q]=size(T);
net=newp(minmax(P),S);
%建立一个有S个输出的感知器网络
[W0]=rands(S,R);
[B0]=rands(S,1);
%初始化给权值，偏差
net.iw{1,1}=W0;
net.b{1}=B0;
%给权值，偏差初始化赋值
net.trainParam.epochs=20;
%定义最大循环次数
net=train(net,P,T);
%进行神经网络的训练
V=[-2 2 -2 2];
%取一数组限制坐标值的大小
plotpv(P,T,V);
%该函数用于在感知器向量图中绘制其要分类的矢量图
axis('equal'),
%令横坐标和纵坐标单位距离相等
title('Input Vector Graph'),
%命名图的标题
xlabel('p1'),
%命名横轴坐标
ylabel('p2'),
%命名纵轴坐标
plotpc(net.iw{1},net.b{1});
%该函数用于绘制感知器的输入矢量的目标向量，即在plotpv中把最终的分界线画出来
