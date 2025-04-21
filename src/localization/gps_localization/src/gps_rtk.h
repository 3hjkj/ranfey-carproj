#include "math.h"
/*******************************************************************************

  File Name   : Radar_CANDATA
  Date        : 2019/11/21
  Version     : 1.0.1
  Desciption  : Radar_CANDATA

  little
*******************************************************************************/


struct Radar_CANDATA
{
 union                        
  {                         
	uint8_t D;
	 struct
		{
		 uint8_t  Length     : 4;
		 uint8_t  Reserved    : 4;
		 }bit;
  }MsgLengh;
  
	union                        
  {                         
	uint32_t D;
	 struct
		{
		 uint32_t  ID    : 32;
		 }bit;
  }MsgID;
 
 union                        
  {                         
	uint8_t D;
	 struct
		{
		 uint8_t  Width    : 5;
		 uint8_t  Reserved    : 3;
		 }bit;
  }DATA0;	

	union                        
  {                         
	uint8_t D;
	 struct
		{
		 uint8_t  Reserved    : 8;
		 }bit;
  }DATA1;

	union                        
  {                         
	uint8_t D;
	 struct
		{
		 uint8_t  Reserved    : 8;
		 }bit;
  }DATA2;

  union                        
  {                         
	uint8_t D;
	 struct
		{
		 uint8_t  Reserved    : 8;
		 }bit;
  }DATA3;


 union                        
  {                         
	uint8_t D;
	 struct
		{
		 uint8_t  Reserved    : 8;
		 }bit;
  }DATA4;

	union                        
  {                         
	uint8_t D;
	 struct
		{
		 uint8_t  Reserved    : 8;
		 }bit;
  }DATA5;		
  
	union                        
  {                         
	uint8_t D;
	 struct
		{
		 uint8_t  LiveCount    : 8;
		 }bit;
  }DATA6;

	union                        
  {                         
	uint64_t D;
	 struct
		{
		 uint64_t  data7    : 8;
		 uint64_t  data6    : 8;
		 uint64_t  data5    : 8;
		 uint64_t  data4    : 8;
		 uint64_t  data3    : 8;
		 uint64_t  data2    : 8;
		 uint64_t  data1    : 8;
		 uint64_t  data0    : 8;
		 }bit;
  }DATA7;
	
};