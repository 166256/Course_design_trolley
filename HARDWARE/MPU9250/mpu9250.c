#include "mpu9250.h"
#include "mpuiic.h"
#include "Filter.h"
#include "math.h"

//#define _DMP_H   //���ʹ��DMP/MPL���㣬��ʹ�øú꣬����DMP��ע�͵�,������main��ȡ����Ӧmpu9250��ʼ��ע��

short aacx,aacy,aacz;	        //���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz;      //������ԭʼ���� 
short mx,my,mz;             //������ԭʼ����
/*IMU��ر���*/
struct imu_para IMU_9250;
struct imu_para Offset_9250;
struct imu_para RC_9250;
struct imu_para Mov_9250;
struct imu_para Fuse_9250;
struct imu_para Calibrated_9250;

/*����У׼*/
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
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
		MPU_Get_Magnetometer(&mx,&my,&mz);	 
}
#endif 

/**************************************************************************
�������ܣ���ʼ��MPU9250������DMP
��ڲ�������  
����  ֵ��0,�ɹ�
**************************************************************************/
u8 MPU9250_Init(void)
{
    u8 res=0;
    IIC_Init();     //��ʼ��IIC����
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);//��λMPU9250
    delay_ms(100);  //��ʱ100ms
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);//����MPU9250
    MPU_Set_Gyro_Fsr(3);					        	//�����Ǵ�����,��2000dps
	  MPU_Set_Accel_Fsr(0);					       	 	//���ٶȴ�����,��2g
	
    MPU_Set_Rate(1000);						       	 	//���ò�����50Hz

    MPU_Write_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0X00);   //�ر������ж�
	  MPU_Write_Byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0X00);//I2C��ģʽ�ر�
	  MPU_Write_Byte(MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	  MPU_Write_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0X82);//INT���ŵ͵�ƽ��Ч������bypassģʽ������ֱ�Ӷ�ȡ������
    res=MPU_Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG);  //��ȡMPU6500��ID
    if(res==MPU6500_ID) //����ID��ȷ
    {
       MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//����CLKSEL,PLL X��Ϊ�ο�,���Բ�����
       MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//���ٶ��������Ƕ�����
		   MPU_Set_Rate(1000);						       	//���ò�����Ϊ50Hz (������),���Ĺٷ��ĵ�
    }else return 1;
 
    res=MPU_Read_Byte(AK8963_ADDR,MAG_WIA);    			//��ȡAK8963 ID   
    if(res==AK8963_ID)
    {
        MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11);		//����AK8963Ϊ���β���ģʽ
    }else return 1;

    return 0;
}

/**************************************************************************
�������ܣ�����MPU9250�����Ǵ����������̷�Χ
					fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
��ڲ�������  
����  ֵ������ֵ:0,���óɹ�
					����,����ʧ�� 
**************************************************************************/
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ  
}

/**************************************************************************
�������ܣ�����MPU9250���ٶȴ����������̷�Χ
					fsr:0,��2g;1,��4g;2,��8g;3,��16g
��ڲ�������  
����  ֵ������ֵ:0,���óɹ�
					����,����ʧ�� 
**************************************************************************/
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}

/**************************************************************************
�������ܣ�����MPU9250�����ֵ�ͨ�˲�����lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
��ڲ�������  
����  ֵ������ֵ:0,���óɹ�
					����,����ʧ��
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
	return MPU_Write_Byte(MPU9250_ADDR,MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}
/**************************************************************************
�������ܣ�����MPU9250�Ĳ�����(�ٶ�Fs=1KHz)
					rate:4~1000(Hz)
��ڲ�������  
����  ֵ������ֵ:0,���óɹ�
					����,����ʧ�� 
**************************************************************************/
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}
/**************************************************************************
�������ܣ��õ��¶�ֵ
��ڲ�������  
����  ֵ���¶�ֵ(������100��)
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
�������ܣ��õ�������ֵ(ԭʼֵ)
��ڲ�����gx,gy,gz:������x,y,z���ԭʼ����(������) 
����  ֵ������ֵ:0,�ɹ�
					����,�������
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
�������ܣ��õ����ٶ�ֵ(ԭʼֵ)
��ڲ�����ax,ay,az:���ٶȼ�x,y,z���ԭʼ����(������)
����  ֵ��0,�ɹ�
					����,�������
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
�������ܣ��õ�������ֵ(ԭʼֵ)
��ڲ�����mx,my,mz:������x,y,z���ԭʼ����(������)
����  ֵ��0,�ɹ�
					����,�������
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
    MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11); //AK8963ÿ�ζ����Ժ���Ҫ��������Ϊ���β���ģʽ
    return res;;
}
/**************************************************************************
�������ܣ�IIC����д
��ڲ�����addr:������ַ reg:�Ĵ�����ַ��len:д�볤�ȣ�buf:������  
����  ֵ��0,����
					����,�������
**************************************************************************/
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
    u8 i;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(IIC_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    for(i=0;i<len;i++)
    {
        IIC_Send_Byte(buf[i]);  //��������
        if(IIC_Wait_Ack())      //�ȴ�ACK
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
} 
/**************************************************************************
�������ܣ�IIC������
��ڲ�����addr:������ַ reg:�Ĵ�����ַ��len:��ȡ���ȣ�buf:������  
����  ֵ��0,����
					����,�������
**************************************************************************/
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(IIC_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC_Wait_Ack();             //�ȴ�Ӧ��
	IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //����������ַ+������
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    while(len)
    {
        if(len==1)*buf=IIC_Read_Byte(0);//������,����nACK 
		else *buf=IIC_Read_Byte(1);		//������,����ACK  
		len--;
		buf++;  
    }
    IIC_Stop();                 //����һ��ֹͣ����
    return 0;       
}
/**************************************************************************
�������ܣ�IICдһ���ֽ�
��ڲ�����addr:������ַ reg:�Ĵ�����ַ��data:����  
����  ֵ��0,����
					����,�������
**************************************************************************/
u8 MPU_Write_Byte(u8 addr,u8 reg,u8 data)
{
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(IIC_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    IIC_Send_Byte(data);        //��������
    if(IIC_Wait_Ack())          //�ȴ�ACK
    {
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}
/**************************************************************************
�������ܣ�IIC��һ���ֽ�
��ڲ�����addr:������ַ reg:�Ĵ�����ַ
����  ֵ��0,����
					����,�������
**************************************************************************/
u8 MPU_Read_Byte(u8 addr,u8 reg)
{
    u8 res;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC_Wait_Ack();             //�ȴ�Ӧ��
	IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //����������ַ+������
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    res=IIC_Read_Byte(0);		//������,����nACK  
    IIC_Stop();                 //����һ��ֹͣ����
    return res;  
}

/*
***************************************************************
*�� �� ��:  �����ʼ��              
*����˵��:  ͨ���ɼ�һ���������ֵ�������������ƫ��ֵ
*��    ��:  ��
*�� �� ֵ:  ��
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
	for (short i = 0; i < 300; ++i)	//2sŪ��
	{
//		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
		MPU_Get_Magnetometer(&mx,&my,&mz);			//�õ�����������
//		Offset_9250.acc_x += aacx;
//		Offset_9250.acc_y += aacy;
//		Offset_9250.acc_z += aacz;
		Offset_9250.gyro_x += gyrox;
		Offset_9250.gyro_y += gyroy;
		Offset_9250.gyro_z += gyroz;
		Offset_9250.mag_x += mx;
		Offset_9250.mag_y += my;
		Offset_9250.mag_z += mz;
		
		delay_ms(10);    // 20ms��ȡһ��
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
*�� �� ��:  ���ݴ���              
*����˵��:  ���ɼ�����ֵת��Ϊʵ������ֵ, ���������ǽ���ȥ��Ư����@�����Դ�Ϊ�������Ͽ������˲�
*���ٶȼƳ�ʼ������ -> ������Χ: ��8g        ��Ӧ������: 4096 LSB/g
*�����ǳ�ʼ������   -> ������Χ: ��2000 dps  ��Ӧ������: 16.4 LSB/dps
*gyro = (gyro_val / 16.4) ��/s = ((gyro_val / 16.4) * PI / 180) rad/s
*��    ��:  ��
*�� �� ֵ:  ��
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
//	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
	MPU_Get_Magnetometer(&mx,&my,&mz);			//�õ�����������
	
	IMU_9250.gyro_x = ((float)gyrox - Offset_9250.gyro_x) * 3.14159 / 16.4f / 180;
	IMU_9250.gyro_y = ((float)gyroy - Offset_9250.gyro_y) * 3.14159 / 16.4f / 180;
	IMU_9250.gyro_z = ((float)gyroz - Offset_9250.gyro_z) * 3.14159 / 16.4f / 180;
	//ȥƫ�ƺ������
	IMU_9250.mag_x = (float)mx / 30;
	IMU_9250.mag_y = (float)my / 30;
	IMU_9250.mag_z = (float)mz / 30;
	
	Calibrated_Mag();
	
	//��ͨ�˲�
	RC_9250.gyro_z = RCFilter(IMU_9250.gyro_z,IMU_RC_gyroz);
	RC_9250.mag_x = RCFilter(Calibrated_9250.mag_x,IMU_RC_magx);
	RC_9250.mag_y = RCFilter(Calibrated_9250.mag_y,IMU_RC_magy);
	//��������
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

	//������ֱ�ӽ��� �Ѵ����0-360
	gyro_angle_dir += Mov_9250.gyro_z * 0.01 * 57.2958f * 1.285 *0.39; // * 1.285
	if (gyro_angle_dir < 0) 
        gyro_angle_dir += 360;
	else if (gyro_angle_dir >= 360) 
	{
		angle_int = (int16_t) gyro_angle_dir % 360;    //�õ�����λ
		angle_decimal = gyro_angle_dir - (int16_t) gyro_angle_dir;    //�õ�С��λ
		gyro_angle_dir = angle_int + angle_decimal;
	}
		//������ֱ�ӽ��� ��Χ��-180��180����û�����0��360
	mag_angle_dir = (atan2((double) Calibrated_9250.mag_y, (double) Calibrated_9250.mag_x)) * 57.2958f - mag_offset_angle;
//	mag_angle_dir = (atan2((double) Calibrated_9250.mag_y, (double) Calibrated_9250.mag_x)) * 57.2958f;
	if (mag_angle_dir < 0) 
        mag_angle_dir += 360;
	else if (mag_angle_dir >= 360) 
	{
		angle_int = (int16_t) mag_angle_dir % 360;    //�õ�����λ
		angle_decimal = mag_angle_dir - (int16_t) mag_angle_dir;    //�õ�С��λ
		mag_angle_dir = angle_int + angle_decimal;
	}
}

/*
***************************************************************
*�� �� ��:  Calibrate_Mag
*����˵��:  �������У׼��������,У׼ֵΪIMU_Calibrated_Mag_vector.x.y.z ...
*��    ��:  ��
*�� �� ֵ:  ��
***************************************************************
*/
void Calibrated_Mag(void) 
{
    //��������
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


