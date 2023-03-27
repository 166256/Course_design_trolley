% 
figure

scatter3(data1(:,1),data1(:,2),data1(:,3));
axis equal
title('Initial Magnetometer Data');

[A,b,expMFS]  = magcal(data1);
xCorrected = (data1-b)*A;


de = HelperDrawEllipsoid;
de.plotCalibrated(A,b,expMFS,data1,xCorrected,'Auto');