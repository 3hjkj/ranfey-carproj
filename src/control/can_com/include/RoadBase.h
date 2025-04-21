#pragma once
#include"map"
#include"glm/glm.hpp"
#include"DataDefines.h"
#include"RoadLink.h"
#include"Function.h"
#include"Signal.h"
#include"Road.h"
using namespace std;
namespace OpenDrive
{
	class RoadBase
	{
	public:
		RoadBase();
		RoadBase(const Road* road);
		RoadBase(const RoadBase & roadBase);
		RoadBase& operator =(const RoadBase & roadBase);
		~RoadBase();
	public:
		string _name;
		string _id;
		double _length;
		PosXY  _initP;
		PosXY _termP;
	};
}

