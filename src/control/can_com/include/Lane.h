#pragma once
#include"DataDefines.h"
#include"LaneLink.h"
#include"LaneWidth.h"
#include"LaneRoadMark.h"
#include"Speed.h"
#include"Height.h"
#include"Function.h"
#include"UserData.h"
namespace OpenDrive
{
	class Lane
	{
	public:
		Lane();
		void SetID(const char *id);
		void SetSectionID(const char*id);
		const char* GetID() const;
		void SetLevel(const char* level);
		void SetType(const EnLaneType &type);
		const EnLaneType GetType() const;
		const char* GetTypeString() const;
		void SetSpeed(Speed *speed);
		const Speed* GetSpeed() const;
		void SetHeight(Height *height);
		const Height* GetHeight() const;
		const LaneLink* GetLink() const;
		void SetType(const char* type);
		void SetLaneReferenceData(const vector<GeoCoordinatePoint>& vecLaneRefernceData);
		void SetLaneBoundaryData(const vector<GeoCoordinatePoint>& vecLaneRefernceData);
		const vector<GeoCoordinatePoint>* GetlaneBoundaryData() const;
		void SetLaneWidth(LaneWidth *width);
		const vector<LaneWidth>* GetLaneWidth() const;
		void SetLink(LaneLink* link);
		void SetRoadMark(LaneRoadMark *roadMark);
		const LaneRoadMark* GetLaneRoadMark() const;
		LaneWidth GetLaneWidthFromLane(const double &soff);
		//LaneWidth GetFirstLaneWidthFromLane(const double &soff);
		void ConstructPolygon();
		Lane& operator= (const Lane &lane);
		const string GetPreLaneID()const;
		const string GetSucLaneID()const;
		Lane(const Lane &lane);
		void CalBoundaryData();
		const double GetLaneSpeed() const;
		const vector<GeoCoordinatePoint>* GetLaneRefernceData() const;

		void SetUserData(UserData *userData);
		const UserData* GetUserData() const;

		~Lane();
	private:
		string _id;
		double _s = 0.0;
		Height *_height;
		EnLaneType _type;
		LaneLink *_link;
		vector<LaneWidth> *_laneWidth;
		LaneRoadMark *_roadMark;
		UserData *_userData;
		Speed *_speed;
		//vector<GeoCoordinatePoint> *_laneReference;
		vector<GeoCoordinatePoint> *_laneReferenceData;
		vector<GeoCoordinatePoint> *_laneBoundaryData;
	public:
		string _level = "false";
	};

}
