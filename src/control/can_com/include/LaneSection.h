#pragma once
#include"map"
#include"Lane.h"
#include"Polygon.h"
#include"Function.h"
using namespace std;

namespace OpenDrive
{
	class LaneSection
	{
	public:
		LaneSection();
		LaneSection(const LaneSection &laneSection);
		LaneSection & operator = (const LaneSection &laneSection);
		void SetLaneSectionID();
		void ConstructPolygon(const vector<glm::dvec3> &vecRoadData);
		void SetID(const char *id);
		void SetWidth(const double &left,const double &right);
		void PushLane(Lane *lane);
		const char* GetID() const;
		const string GetLaneID(const char* sectionID, const double & s, const double &t, const double &distance) const;
		const string GetLaneID(const char* sectionID, const double & longitude, const double &latitude, const double & x, const double & y, const double & hdg) const;
		vector<Lane*> GetAllLane() const ;

		Lane* GetLane(const char* laneID);

		const string GetPreLaneID(const char* laneID)const;
		const string GetSucLaneID(const char* laneID)const;
		const vector<string> GetLeftLaneIDS(const char* laneID)const;
		const vector<string> GetRightLaneIDS(const char* laneID)const;
		const vector<string> GetAllDrivingLaneIDS() const;
		const vector<string> GetAllLaneIDS() const;
		bool IsInSideLaneSection(const double & longitude, const double &latitude);
		//const vector<LaneWidth>* GetFirstLaneWith(const char* laneID);
		const LaneWidth GetLaneWith(const char* laneID, double s);
		const vector<RoadWidth> GetSectionWidthofRoad() const;
		const NearestDistance GetNeartestRoadandLaneAttributes(const char* laneID, const double &s, const double &t) const;
		const NearestRoadMark GetNeartestLaneRoadMark(const char* laneID) const;
		void CalBoundaryData(const char* laneID);
		const vector<GeoCoordinatePoint>* GetlaneBoundaryData(const char* laneID) const;
		const double GetLaneSpeed(const char* laneID) const;
		const vector<PosXY>* GetLeftLinePosFromLane(const char* laneID) const;
		const vector<PosXY>* GetRightLinePosFromLane(const char* laneID) const;
		~LaneSection();
	private:
		map<string, Lane*>* _mapLane;
		vector<Polygon*> *_vecPlygon;
		string _id;
		double _s;
		double _leftWidth;
		double _rightWidth;
	};
}

