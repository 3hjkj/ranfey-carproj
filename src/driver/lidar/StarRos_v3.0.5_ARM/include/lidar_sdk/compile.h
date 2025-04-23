#ifndef __COMPILE__
#define  __COMPILE__

#include <vector>
#include <map>
#include <algorithm>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include<stdio.h>
//#include<memory.h>
using namespace std;

#define PACKETSIZE 0X40000
#define BUFFSIZE (0X40000*5)
#define SLOW_VEC_SIZE 500
#define PPS_STAMP_DEFAULT (0x80000000) //max T0,2^31, 2147483648
#define PPS_DIFF_DEFAULT  (5000000)
#define UTC_STAMP_DEFAULT (0.0)
#define T0_STEP (0x80000000) // 2^31, 2147483648
#define HEADER_SIZE 0x14000
const int HEADSIZE = 81920;

#ifndef __dllexport
#if defined(_MSC_VER)
#define __dllexport __declspec(dllexport)
#define __dllimport __declspec(dllimport)
#define __dllhidden
#elif defined(__MINGW32__) || defined(__SYGWIN__)
#define __dllexport __attribute__((dllexport))
#define __dllimport __attribute__((dllimport))
#define __dllhidden __attribute__((visibility("hidden")))
#else
#define __dllexport
#define __dllimport
#define __dllhidden
#endif
#endif

typedef vector<int> INTV;
typedef vector<double> DBV;
typedef vector<DBV> DBVV;
typedef unsigned char BYTE;
typedef vector <BYTE> BYTEV;
typedef map<int, vector<float>> cfgDATA;
struct DataBuff_S
{
	char* _ptr;//数组的首地址
	int buff_capacity;//buff的容量
	int cur_size;//目前buff的有效size
	DataBuff_S()
	{
		_ptr = NULL;
		buff_capacity = 0;
		cur_size = 0;
	}
};

#endif // __PUBTYPE__
