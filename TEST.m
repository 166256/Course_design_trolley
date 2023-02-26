clc;clear

M(1000,4) = [0];
%% 接收数据
for i = 1:1000 %循环读取 ，1000个数据
    out = [0,1,2,3];
%     M(i+1,:) = [M(i+1,:);out];
    M(i,:) = out;
     
    i=i+1;
end

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