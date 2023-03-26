#include "track.h"
#include "Tim_encoder_speed.h"
#include "stdio.h"

#define GEO_ANGLE(x)    ((x) * 0.0174533f)  // PI / 180 = 0.0174533f

float north_distance = 0,east_distance = 0;
int north_distance_cm = 0,east_distance_cm = 0;
float north_v,east_v,v;

void track_construction(float angle,float ms)
{
	angle = GEO_ANGLE(angle); // 将角度转换成弧度制
	v = (rotateSpeed_R > rotateSpeed_L ? rotateSpeed_L : rotateSpeed_R);
	north_v = v * cos(angle);
	east_v = v * sin(angle);
	float t = ms / 1000;
	north_distance += north_v * t;
	east_distance += east_v * t;
	north_distance_cm = north_distance * 100;
	east_distance_cm = east_distance * 100;	
	
//	north_distance_cm += north_v * ms / 10;
//	east_distance_cm += east_v * ms / 10;
}
