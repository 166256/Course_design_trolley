#include "mpu9250.h"
#include "mpuiic.h"
#include "Filter.h"
#include "math.h"

//#define _DMP_H   //如果使用DMP/MPL解算，则使用该宏，不用DMP则注释掉,另外在main中取消对应mpu9250初始化注释

short aacx,aacy,aacz;	        //加速度传感器原始数据
short gyrox,gyroy,gyroz;      //陀螺仪原始数据 
short mx,my,mz;             //磁力计原始数据
/*IMU相关变量*/
struct imu_para IMU_9250;
struct imu_para Offset_9250;
struct imu_para RC_9250;
struct imu_para Mov_9250;
struct imu_para Fuse_9250;
struct imu_para Calibrated_9250;

/*椭球校准*/
float mag_offset[3] = {0.762330522915023,	5.62922049534486,	5.59342855296572};
float Calibrated_Mag_Matrix[3][3] = {
	{0.945490777763641,	0.0275058132534751,	-0.0855992779350727},
	{0.027505813253475,	1.02716917328571,	0.0667401429588575},
	{-0.08559927793507,	0.0667401429588575,	1.04289845908036},
};	

#ifdef _DMP_H
void Read_DMP(void)
{	
	 if(mpu_mpl_get_data(&pitch,&roll,&yaw)==0)
 { 
 } 	
} 
#else
 void Read_DMP(void)
{
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
		MPU_Get_Magnetometer(&mx,&my,&mz);	 
}
#endif 

/**************************************************************************
函数功能：初始化MPU9250，不用DMP
入口参数：无  
返回  值：0,成功
**************************************************************************/
u8 MPU9250_Init(void)
{
    u8 res=0;
    IIC_Init();     //初始化IIC总线
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);//复位MPU9250
    delay_ms(100);  //延时100ms
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);//唤醒MPU9250
    MPU_Set_Gyro_Fsr(3);					        	//陀螺仪传感器,±2000dps
	  MPU_Set_Accel_Fsr(0);					       	 	//加速度传感器,±2g
	
    MPU_Set_Rate(1000);						       	 	//设置采样率50Hz

    MPU_Write_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0X00);   //关闭所有中断
	  MPU_Write_Byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0X00);//I2C主模式关闭
	  MPU_Write_Byte(MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	  MPU_Write_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0X82);//INT引脚低电平有效，开启bypass模式，可以直接读取磁力计
    res=MPU_Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG);  //读取MPU6500的ID
    if(res==MPU6500_ID) //器件ID正确
    {
       MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//设置CLKSEL,PLL X轴为参考,可以不设置
       MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//加速度与陀螺仪都工作
		   MPU_Set_Rate(1000);						       	//设置采样率为50Hz (陀螺仪),查阅官方文档
    }else return 1;
 
    res=MPU_Read_Byte(AK8963_ADDR,MAG_WIA);    			//读取AK8963 ID   
    if(res==AK8963_ID)
    {
        MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11);		//设置AK8963为单次测量模式
    }else return 1;

    return 0;
}

/**************************************************************************
函数功能：设置MPU9250陀螺仪传感器满量程范围
					fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
入口参数：无  
返回  值：返回值:0,设置成功
					其他,设置失败 
**************************************************************************/
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}

/**************************************************************************
函数功能：设置MPU9250加速度传感器满量程范围
					fsr:0,±2g;1,±4g;2,±8g;3,±16g
入口参数：无  
返回  值：返回值:0,设置成功
					其他,设置失败 
**************************************************************************/
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}

/**************************************************************************
函数功能：设置MPU9250的数字低通滤波器，lpf:数字低通滤波频率(Hz)
入口参数：无  
返回  值：返回值:0,设置成功
					其他,设置失败
**************************************************************************/
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU9250_ADDR,MPU_CFG_REG,data);//设置数字低通滤波器  
}
/**************************************************************************
函数功能：设置MPU9250的采样率(假定Fs=1KHz)
					rate:4~1000(Hz)
入口参数：无  
返回  值：返回值:0,设置成功
					其他,设置失败 
**************************************************************************/
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}
/**************************************************************************
函数功能：得到温度值
入口参数：无  
返回  值：温度值(扩大了100倍)
**************************************************************************/
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU9250_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=21+((double)raw)/333.87;  
    return temp*100;;
}

/**************************************************************************
函数功能：得到陀螺仪值(原始值)
入口参数：gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号) 
返回  值：返回值:0,成功
					其他,错误代码
**************************************************************************/
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res=0; 
	res=MPU_Read_Len(MPU9250_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[2]<<8)|buf[3];  
		*gy=((u16)buf[0]<<8)|buf[1];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
/**************************************************************************
函数功能：得到加速度值(原始值)
入口参数：ax,ay,az:加速度计x,y,z轴的原始读数(带符号)
返回  值：0,成功
					其他,错误代码
**************************************************************************/
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res=0;  
	res=MPU_Read_Len(MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
/**************************************************************************
函数功能：得到磁力计值(原始值)
入口参数：mx,my,mz:磁力计x,y,z轴的原始读数(带符号)
返回  值：0,成功
					其他,错误代码
**************************************************************************/
u8 MPU_Get_Magnetometer(short *mx,short *my,short *mz)
{
    u8 buf[6],res=0;  
	res=MPU_Read_Len(AK8963_ADDR,MAG_XOUT_L,6,buf);
	if(res==0)
	{
		*mx=((u16)buf[1]<<8)|buf[0];  
		*my=((u16)buf[3]<<8)|buf[2];  
		*mz=((u16)buf[5]<<8)|buf[4];
	} 	
    MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11); //AK8963每次读完以后都需要重新设置为单次测量模式
    return res;;
}
/**************************************************************************
函数功能：IIC连续写
入口参数：addr:器件地址 reg:寄存器地址，len:写入长度，buf:数据区  
返回  值：0,正常
					其他,错误代码
**************************************************************************/
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
    u8 i;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_Wait_Ack())          //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
    for(i=0;i<len;i++)
    {
        IIC_Send_Byte(buf[i]);  //发送数据
        if(IIC_Wait_Ack())      //等待ACK
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
} 
/**************************************************************************
函数功能：IIC连续读
入口参数：addr:器件地址 reg:寄存器地址，len:读取长度，buf:数据区  
返回  值：0,正常
					其他,错误代码
**************************************************************************/
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_Wait_Ack())          //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
	IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //发送器件地址+读命令
    IIC_Wait_Ack();             //等待应答
    while(len)
    {
        if(len==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++;  
    }
    IIC_Stop();                 //产生一个停止条件
    return 0;       
}
/**************************************************************************
函数功能：IIC写一个字节
入口参数：addr:器件地址 reg:寄存器地址，data:数据  
返回  值：0,正常
					其他,错误代码
**************************************************************************/
u8 MPU_Write_Byte(u8 addr,u8 reg,u8 data)
{
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_Wait_Ack())          //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
    IIC_Send_Byte(data);        //发送数据
    if(IIC_Wait_Ack())          //等待ACK
    {
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}
/**************************************************************************
函数功能：IIC读一个字节
入口参数：addr:器件地址 reg:寄存器地址
返回  值：0,正常
					其他,错误代码
**************************************************************************/
u8 MPU_Read_Byte(u8 addr,u8 reg)
{
    u8 res;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    IIC_Wait_Ack();             //等待应答
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
	IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //发送器件地址+读命令
    IIC_Wait_Ack();             //等待应答
    res=IIC_Read_Byte(0);		//读数据,发送nACK  
    IIC_Stop();                 //产生一个停止条件
    return res;  
}

/*
***************************************************************
*函 数 名:  较零初始化              
*功能说明:  通过采集一定数据求均值计算陀螺仪零点偏移值
*形    参:  无
*返 回 值:  无
***************************************************************
*/
void IMU_OffsetInit(void)     
{
	Offset_9250.acc_x = 0;
	Offset_9250.acc_y = 0;
	Offset_9250.acc_z = 0;
	
	Offset_9250.gyro_x = 0;
	Offset_9250.gyro_y = 0;
	Offset_9250.gyro_z = 0;
	for (short i = 0; i < 300; ++i)	//2s弄完
	{
//		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
		MPU_Get_Magnetometer(&mx,&my,&mz);			//得到磁力计数据
//		Offset_9250.acc_x += aacx;
//		Offset_9250.acc_y += aacy;
//		Offset_9250.acc_z += aacz;
		Offset_9250.gyro_x += gyrox;
		Offset_9250.gyro_y += gyroy;
		Offset_9250.gyro_z += gyroz;
		Offset_9250.mag_x += mx;
		Offset_9250.mag_y += my;
		Offset_9250.mag_z += mz;
		
		delay_ms(10);    // 20ms读取一次
	}

//	Offset_9250.acc_x  /= 100;
//	Offset_9250.acc_y  /= 100;
//	Offset_9250.acc_z  /= 100;

	Offset_9250.gyro_x /= 300;
	Offset_9250.gyro_y /= 300;
	Offset_9250.gyro_z /= 300;
	
	Offset_9250.mag_x /= 9000;
	Offset_9250.mag_y /= 9000;
	Offset_9250.mag_z /= 9000;
	
	Calibrated_9250.mag_x =
					(Offset_9250.mag_x - mag_offset[0]) * Calibrated_Mag_Matrix[0][0] +
					(Offset_9250.mag_y - mag_offset[1]) * Calibrated_Mag_Matrix[1][0] +
					(Offset_9250.mag_z - mag_offset[2]) * Calibrated_Mag_Matrix[2][0];
	Calibrated_9250.mag_y =
					(Offset_9250.mag_x - mag_offset[0]) * Calibrated_Mag_Matrix[0][1] +
					(Offset_9250.mag_y - mag_offset[1]) * Calibrated_Mag_Matrix[1][1] +
					(Offset_9250.mag_z - mag_offset[2]) * Calibrated_Mag_Matrix[2][1];
	
	mag_offset_angle = (atan2((double) Calibrated_9250.mag_y, (double) Calibrated_9250.mag_x))* 57.2958f;
}

/*
******************************************************************************************
*函 数 名:  数据处理              
*功能说明:  将采集的数值转化为实际物理值, 并对陀螺仪进行去零漂处理@可以以此为基础加上卡尔曼滤波
*加速度计初始化配置 -> 测量范围: ±8g        对应灵敏度: 4096 LSB/g
*陀螺仪初始化配置   -> 测量范围: ±2000 dps  对应灵敏度: 16.4 LSB/dps
*gyro = (gyro_val / 16.4) °/s = ((gyro_val / 16.4) * PI / 180) rad/s
*形    参:  无
*返 回 值:  无
*******************************************************************************************
*/
struct RC_Para IMU_gyroz_Para = {0, 0, 0.4};
struct RC_Para IMU_magx_Para = {0, 0, 0.6};
struct RC_Para IMU_magy_Para = {0, 0, 0.6};
struct RC_Para IMU_magz_Para = {0, 0, 0.6};
RC_Filter_pt IMU_RC_gyroz = &IMU_gyroz_Para;
RC_Filter_pt IMU_RC_magx = &IMU_magx_Para;
RC_Filter_pt IMU_RC_magy = &IMU_magy_Para;
RC_Filter_pt IMU_RC_magz = &IMU_magz_Para;

float IMU_MovAverbuf_gyroz[Windows + 1] = {0};
float IMU_MovAverbuf_magx[Windows + 1] = {0};
float IMU_MovAverbuf_magy[Windows + 1] = {0};
float IMU_MovAverbuf_magz[Windows + 1] = {0};

void IMU_9250_GetValues(void)
{
//	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
	MPU_Get_Magnetometer(&mx,&my,&mz);			//得到磁力计数据
	
	IMU_9250.gyro_x = ((float)gyrox - Offset_9250.gyro_x) * 3.14159 / 16.4f / 180;
	IMU_9250.gyro_y = ((float)gyroy - Offset_9250.gyro_y) * 3.14159 / 16.4f / 180;
	IMU_9250.gyro_z = ((float)gyroz - Offset_9250.gyro_z) * 3.14159 / 16.4f / 180;
	//去偏移后的数据
	IMU_9250.mag_x = (float)mx / 30;
	IMU_9250.mag_y = (float)my / 30;
	IMU_9250.mag_z = (float)mz / 30;
	
	Calibrated_Mag();
	
	//低通滤波
	RC_9250.gyro_z = RCFilter(IMU_9250.gyro_z,IMU_RC_gyroz);
	RC_9250.mag_x = RCFilter(Calibrated_9250.mag_x,IMU_RC_magx);
	RC_9250.mag_y = RCFilter(Calibrated_9250.mag_y,IMU_RC_magy);
	//滑动窗口
	Mov_9250.gyro_z = Movingaverage_filter(RC_9250.gyro_z, IMU_MovAverbuf_gyroz, Windows);
	Mov_9250.mag_x = Movingaverage_filter(RC_9250.mag_x, IMU_MovAverbuf_magx, Windows);
	Mov_9250.mag_y = Movingaverage_filter(RC_9250.mag_y, IMU_MovAverbuf_magy, Windows);
	
}

float mag_angle_dir;
float mag_offset_angle;
float gyro_angle_dir;
float fuse_angle;
void get_angle_IMU(void)
{
	short angle_int;
	float angle_decimal;

	//陀螺仪直接结算 已处理成0-360
	gyro_angle_dir += Mov_9250.gyro_z * 0.01 * 57.2958f * 1.285 *0.39; // * 1.285
	if (gyro_angle_dir < 0) 
        gyro_angle_dir += 360;
	else if (gyro_angle_dir >= 360) 
	{
		angle_int = (int16_t) gyro_angle_dir % 360;    //得到整数位
		angle_decimal = gyro_angle_dir - (int16_t) gyro_angle_dir;    //得到小数位
		gyro_angle_dir = angle_int + angle_decimal;
	}
		//磁力计直接解算 范围是-180到180，还没处理成0到360
	mag_angle_dir = (atan2((double) Calibrated_9250.mag_y, (double) Calibrated_9250.mag_x)) * 57.2958f - mag_offset_angle;
//	mag_angle_dir = (atan2((double) Calibrated_9250.mag_y, (double) Calibrated_9250.mag_x)) * 57.2958f;
	if (mag_angle_dir < 0) 
        mag_angle_dir += 360;
	else if (mag_angle_dir >= 360) 
	{
		angle_int = (int16_t) mag_angle_dir % 360;    //得到整数位
		angle_decimal = mag_angle_dir - (int16_t) mag_angle_dir;    //得到小数位
		mag_angle_dir = angle_int + angle_decimal;
	}
}

/*
***************************************************************
*函 数 名:  Calibrate_Mag
*功能说明:  椭球拟合校准电子罗盘,校准值为IMU_Calibrated_Mag_vector.x.y.z ...
*形    参:  无
*返 回 值:  无
***************************************************************
*/
void Calibrated_Mag(void) 
{
    //矩阵运算
    Calibrated_9250.mag_x =
            (IMU_9250.mag_x - mag_offset[0]) * Calibrated_Mag_Matrix[0][0] +
            (IMU_9250.mag_y - mag_offset[1]) * Calibrated_Mag_Matrix[1][0] +
            (IMU_9250.mag_z - mag_offset[2]) * Calibrated_Mag_Matrix[2][0];
    Calibrated_9250.mag_y =
            (IMU_9250.mag_x - mag_offset[0]) * Calibrated_Mag_Matrix[0][1] +
            (IMU_9250.mag_y - mag_offset[1]) * Calibrated_Mag_Matrix[1][1] +
            (IMU_9250.mag_z - mag_offset[2]) * Calibrated_Mag_Matrix[2][1];
    Calibrated_9250.mag_z =
            (IMU_9250.mag_x - mag_offset[0]) * Calibrated_Mag_Matrix[0][2] +
            (IMU_9250.mag_y - mag_offset[1]) * Calibrated_Mag_Matrix[1][2] +
            (IMU_9250.mag_z - mag_offset[2]) * Calibrated_Mag_Matrix[2][2];
}


