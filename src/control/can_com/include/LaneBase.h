#pragma once
#include"Lane.h"
#include"DataDefines.h"
#include"LaneLink.h"
#include"LaneWidth.h"
#include"LaneRoadMark.h"
#include"Speed.h"
#include"Function.h"
#include"string"
using namespace std;
namespace OpenDrive
{
	class LaneBase
	{
	public:
		LaneBase();
		LaneBase(const Lane*lane);
		LaneBase& operator= (const LaneBase &lane);
		LaneBase(const LaneBase &lane);
		const double GetLaneSpeed() const;
		//const vector<LaneWidth>& GetLaneWidth() const;
		void SetRoadID(const char* roadID);
		void SetSectionID(const char* roadID);
		~LaneBase();
	public:
		string _id;
		string _RoadID;
		string _SectionID;
		PosXY  _initP;
		PosXY _termP;
		double _length = 0.0;
		EnLaneType _type;
		Speed *_speed;
		LaneRoadMark *_roadMark;
		vector<LaneWidth> *_vecLaneWidth;
		string _strMaterial = "";
		string _strVisible = "";
		string _strAccess = "";
		string _strHeight = "";
		string _strRule = "";
	};
}

