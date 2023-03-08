%% 连接与配置
delete(instrfindall); %删除串口所有信息，方便下次使用  
s=serial('com16'); %选择串口号  
set(s,'BaudRate',115200,'StopBits',1,'Parity','none');%设置波特率  停止位  校验位  
fopen(s);%打开串口
%  fwrite(s,100,'uint8');%向单片机发送握手信号 

M(1000,4) = [0];
%% 接收数据
for i = 1:100 %循环读取 ，1000个数据
    out = fread(s,4,'uint8')%读取 数据个数 与 类型  
%    a(i)=typecast(fliplr(uint8([out(4) out(3) out(2) out(1)])), 'single'); %将4个8位整形和成一个浮点型
    
    M(i,:) = out;
%     pause(0.01);%延时一小段时间，放在接受过快，数据丢失 
    i=i+1;
end

%% 写入数据到txt文件
fid = fopen('data.txt','wt');	% data.txt为写入文件名
matrix = M;                     % M为要存储的矩阵
[m,n]=size(matrix);                      
 for i=1:1:m
   for j=1:1:n
      if j==n
        fprintf(fid,'%f\n',matrix(i,j));
     else
       fprintf(fid,'%f\t',matrix(i,j));
      end
   end
end
fclose(fid);

%% 画图
% plot(a);%画图
% hold on;%使下次画的图不被覆盖，方便与上次图形进行对比
% axis([0,500,-3,3]);%设置x，y轴坐标范围  

%% 关闭串口
fclose(s) ;%关闭串口

