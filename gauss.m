% clc;clear;

%% 数据点拉伸
% img = load('data.mat'); % 下载数据
img1 = img;

res= double(img1); % 将数据转换为double类型
[cols, rows] = size(res); % 丈量数据矩阵尺寸，此处数据集为1行1280列
resImg = imresize(res, [1, cols+1], 'bilinear'); % 将数据集插值拉伸为1行1281列，差值方式为线性插值

%% X轴平移至关于原点对称的区间
bias = round(size(resImg, 2), 2); % 平移量
xdata = 1:size(resImg,2); % X轴平以前的坐标区间
xdata = xdata - bias; % X轴平移后的坐标区间，原1281个坐标点以平移至[-640,640]的对称区间
ydata = resImg; % Y轴为数据点的值

%% Y轴数据归一化
y1 = ydata/max(ydata); % Y轴数据归一化
x1 = xdata; % X轴数据保持不变

%% 高斯拟合
fun = fittype('a*exp(-((x-b)/c)^2)+d'); % 确定高斯函数的表达形式
% 其中a是振幅，b是期望，c是标准差，d是直流分量
sP_idx = [round(0.2*numel(x1)), round(0.4*numel(x1)), ...
          round(0.6*numel(x1)), round(0.8*numel(x1))]; % 选取X轴上等间距的四个点
startPoints = x1(sP_idx); % 将上述4个点设置为'Start'参数
[cf,gof] = fit(x1(:),y1(:),fun,'Start',startPoints); % 高斯拟合
yy = cf.a*exp( -((x1-cf.b)/cf.c).^2 )+cf.d; %得到拟合后的高斯曲线
 
%% 绘图
figure; plot(x1, y1, 'b.');  % 绘制数据点
hold on; plot(x1 ,yy, 'r');  % 绘制拟合的高斯曲线
legend('原始数据', '拟合数据')
hold off;

%% 输出高斯拟合得到的参数
A = cf.a * max(Ydata); % 幅值
mu = cf.b + bias; % 期望
sigma = cf.c; % 标准差
