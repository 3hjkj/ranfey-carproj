#pragma once
#include <memory>
#include <string.h>
union LidarPoint
{
	float data[12];
	struct
	{
		float x;         //x
		float y;         //y
		float z;         //z
		float intensity; //强度
		float hangle;    //水平角
		float range; //距离
		int laserid;     //线号
		float timeflag;  //时间戳	

		union
		{
			unsigned char rgba[4];
			union {
				unsigned char r;
				unsigned char g;
				unsigned char b;
				unsigned char a;
			};
		};
		unsigned int frameid;
	};

	LidarPoint(){
		memset(data, 0, 48);
		a = 255;
	}
};

union PosePoint {
	float data[8];
	struct
	{
		float x;
		float y;
		float z;
		float roll;
		float pitch;
		float yaw;
		float timeflag;
		float frameid;
	};
};

/**
*	IMU数据表示
*/
union ImuPoint
{
	float data[12];
	struct
	{
		float timeflag;
		float acc_x;
		float acc_y;
		float acc_z;
		float ang_velo_x;
		float ang_velo_y;
		float ang_velo_z;
		float roll;
		float pitch;
		float yaw;
	};
};

union GPSPoint
{
	float data[8];
	struct
	{
		float timestamp;
		float x;
		float y;
		float z;
		float covariance_x;
		float covariance_y;
		float covariance_z;
	};
};

#define GRID_SIZE_SLAM  128000           //(360/0.09)*32    最小角度分辨率0.01度
typedef struct SLAM_GRIDE {
	int ptNum;
	LidarPoint *gridPt;
	//fancy::slam::LidarPoint *gridPt[GRID_SIZE_SLAM];	
	SLAM_GRIDE() {
		ptNum = GRID_SIZE_SLAM;
		gridPt = new LidarPoint[GRID_SIZE_SLAM];
		memset(gridPt, 0, sizeof(LidarPoint)* GRID_SIZE_SLAM);
	}
	~SLAM_GRIDE() {
		delete[] gridPt;
		gridPt = NULL;
	}
}SLAM_GRIDE_S;