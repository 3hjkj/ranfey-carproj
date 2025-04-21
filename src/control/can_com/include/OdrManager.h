#pragma once

#include"Road.h"
#include"Junction.h"
#include"unordered_map"
#include"xml/tinyxml.h"
#include"xml/tinystr.h"
#include"glm/glm.hpp"
#include"DataDefines.h"
#include"Elevation.h"
#include"SignalReference.h"
#include"NavinfoGISCoord.h"
#include"LaneBase.h"
#include"RoadBase.h"
#include"stack"
#include"Grid.h"
#include<fstream>
#include<iostream>
using namespace std;
namespace OpenDrive
{
	class  OdrManager
	{
	public:
		explicit OdrManager();

		bool LoadFile(const char* name);
		const vector<OdrInfo> GetOdrInfo(const double & longitude, const double &latitude) const;
		const vector<OdrInfo> GetOdrInfoByXY(const double & x, const double &y) const;
		bool  GetSucRoadID(const char* roadID, char**&vecRoadID, int &nSize);
		bool  GetPreRoadID(const char* roadID, char**&vecRoadID, int &nSize);
		bool  GetPreLaneID(const char* roadID, const char* sectionID, const char* laneID, char**&vecLane, int &nSize);
		bool  GetSucLaneID(const char* roadID, const char* sectionID, const char* laneID, char**&vecLane, int &nSize);
		bool  GetPreLaneRelationID(const char* roadID, const char* sectionID, const char* laneID, LaneRelation*&vecLaneRelation, int &nSize);
		
		bool  GetSucLaneRelationID(const char* roadID, const char* sectionID, const char* laneID, LaneRelation*&vecLaneRelation, int &nSize);
		bool  GetLeftLaneIDS(const char* roadID, const char* sectionID, const char* laneID, char**&vecLane, int &nSize);
		bool  GetRightLaneIDS(const char* roadID, const char* sectionID, const char* laneID, char**&vecLane, int &nSize);
		bool  GetAllDrivingLaneIDS(const char* roadID, const char* sectionID, char**&vecLane, int &nSize);
		bool  GetAllSectionIDS(const char* roadID, char**&vecSection, int &nSize);
		bool  GetAllRoadIDS(char**&vecRoad, int &nSize);
		bool GetSectionWidthofRoad(const char* roadID, const char* sectionID, RoadWidth*&vecRoaDWidth, int &nSize);
		//const vector<RoadWidth> GetSectionWidthofRoad(const char* roadID, const char* sectionID) const;

		bool  GetLanePosiontAndDircetions(const char* roadID, const char* sectionID, const char* laneID, const double & longitude, const double &latitude, Position*& pos, int &nSize);
		bool  GetLanePosiontAndDircetionsByXY(const char* roadID, const char* sectionID, const char* laneID, const double &x, const double &y, Position*& pos, int &nSize);
		bool  GetLaneTurningAttributes(const char* roadID, const char* sectionID, const char* laneID, const double &headDir, TurningAttr*& turAtt, int &nSize);


		 NearestDistance* GetNeartestRoadandLaneAttributes(const double & longitude, const double &latitude);
		 NearestDistance* GetNeartestRoadandLaneAttributesByXY(const double & x, const double &y);

		NearestRoadMark GetNeartestLaneRoadMark(const double & longitude, const double &latitude);
		NearestRoadMark GetNeartestLaneRoadMarkByXY(const double &x, const double &y);
		bool GetEdgePolygon(const double &longitude, const double &latitude, const double &radius, PosXY**& pos, int &nSize, int *&length);
		bool GetEdgePolygonByXY(const double &x, const double &y, const double &radius, PosXY**& pos, int &nSize, int *&length);
		//const RoadProperty GetLaneProperty(const vector<Position> &vecPosition) const;
		bool GetEdgePolygon(const double &longitude, const double &latitude, const double &radius, PosXY*& pos, int &nSize);
		bool GetEdgePolygonByXY(const double &x, const double &y, const double &radius, PosXY*& pos, int &nSize);
		const double GetLaneSpeed(const char* roadID, const char* sectionID, const char* laneID) const; //m/s;
		bool GetLightPos(const double &longitude, const double &latitude, SignalPos*&pos, int &nSize) ;
		bool GetLightPosByXY(const double &x, const double &y, SignalPos*&pos, int &nSize) ;
		bool GetStopLinePos(const double &longitude, const double &latitude, SignalPos*&pos, int &nSize) ;
		bool GetStopLinePosByXY(const double &x, const double &y, SignalPos*&pos, int &nSize) ;
		bool GetLeftLinePosFromLane(const char* roadID, const char* sectionID, const char* laneID, const double &longitude, const double &latitude, PosXY*& pos, int &nSize); //车道左边界的相对坐标;
		bool GetLeftLinePosFromLaneByXY(const char* roadID, const char* sectionID, const char* laneID, const double &x, const double &y, PosXY*& pos, int &nSize); //车道左边界的相对坐标;
		bool GetRightLinePosFromLane(const char* roadID, const char* sectionID, const char* laneID, const double &longitude, const double &latitude, PosXY*& pos, int &nSize);//车道右边界的相对坐标;
		bool GetRightLinePosFromLaneByXY(const char* roadID, const char* sectionID, const char* laneID, const double &x, const double &y, PosXY*& pos, int &nSize);//车道右边界的相对坐标;
		PosXY GetXYFromRoadID(const char* roadID, const double &s, const double &t);
		PosXY GetSTFromRoadID(const char* roadID, const double &x, const double &y);
		PosXY LatLonToUTM(const double &lon, const double &lat) const;
		PosXY XYToLatLonUTM(const double &lon, const double &lat) const;
		LaneBase* GetLaneBase(const char* roadID, const char* sectionID, const char* laneID);
		RoadBase* GetRoadBase(const char* roadID);
		RoadDrivingAttr GetRoadDriveTurningAttr(const char* roadID);
		void PreRoadID(const char* roadID, const double &length, const double &dir, stack<string> &stkRoad);
		bool GetLaneInfor(const double &longitude, const double &latitude, const double &dir, const double &length, const double &width, bool bRoad_Edge, PosXY**& pos, int &nArraySize, int *&nSize, const double& thresholdDegreeValue);
		bool GetLaneInforByXY(const double &x, const double &y, const double &dir, const double &length , const double &width, bool bRoad_Edge, PosXY**& pos, int &nArraySize, int *&nSize, const double& thresholdDegreeValue);
		void GetRoadLaneInfor(Road*road, vector<LaneGeoCoordinage> &vecGeo, const set<string> &setTerm, set<string> &hasRoad, const vector<glm::dvec2> &vecData);
		~OdrManager();
	private:
		vector<vector<glm::dvec3>> GetPrePolygon(const double &s, Road* road);
		vector<vector<glm::dvec3>> GetSucPolygon(Road* road, const double &len);
		vector<vector<glm::dvec3>> GetPolygon(const double &s, Road* road, const double &length);
	private:
		unordered_map<string, Road*> *_mapRoad;
		unordered_map<string, Junction*> *_mapJunction;

	};

}

