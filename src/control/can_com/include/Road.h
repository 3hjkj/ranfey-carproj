#pragma once
#include "Lane.h"
#include "LaneSection.h"
#include"unordered_map"
#include"glm/glm.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include"DataDefines.h"
#include"Geometry.h"
#include"RoadLink.h"
#include"Polygon.h"
#include"Function.h"
#include"Elevation.h"
#include"Signal.h"
#include"LaneOffset.h"
#include "Object.h"
#include <string> 
//#include <io.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include<ctype.h>
using namespace std;
namespace OpenDrive
{
	
	//extern void QuickSort(vector<double> &vecData);
	//extern string DoubleToString(const double &value);
	//extern string IntToString(const long int &value);

	class Road
	{
	public:
		Road();
		Road(const Road & obj3d);
		Road& operator =(const Road & road);
		void SetID(const char* id);
		void SetName(const char* name);
		void SetJunction(const char* junction);
		void SetLength(const double &junction);
		void SetLink(const RoadLink & link, const char* name);
		const char* GetSucRoad() const;
		const char* GetPreRoad() const;
		void PushbackLaneSection(LaneSection *laneSection);
		LaneSection* GetSection(const char* sectionID);
		void PushbackData(Geometry *geo);
		void PushElevation(Elevation *ele);
		void PushLaneOffSet(LaneOffset *laneOffset);
		void PushSignal(Signal *sigl);
		void PushObject(Object *obj);
		void ResizeData();
		void ResizeLaneSection();
		const char* GetID() const;
		const double GetLength() const;
		const char* GetName() const;
		const glm::dvec3 GetInitPosXY() const;
		const glm::dvec3 GetTermPosXY() const;
		void ConstructPolygon(glm::dvec2 &max, glm::dvec2 &min);
		bool IsInSideRoad(const double & longitude, const double &latitude);
		//void GetRefernceData();
		const string GetPreLaneID(const char* sectionID, const char* laneID) const;
		const string GetSucLaneID(const char* sectionID, const char* laneID) const;
		const string GetLaneSectionID(const double &s) const;
		const string GetLaneID(const char* sectionID, const double & longitude, const double &latitude, const double &distance) const;
		const string GetLaneID(const char* sectionID, const double & longitude, const double &latitude) const;
		const vector<string> GetLeftLaneIDS(const char* sectionID, const char* laneID)const;
		const vector<string> GetRightLaneIDS(const char* sectionID, const char* laneID)const;
		const vector<string> GetAllDrivingLaneIDS(const char* sectionID) const;
		const vector<string> GetAllSectionIDS() const;
		const vector<RoadWidth> GetSectionWidthofRoad(const char* sectionID) const;
		const vector<Position> GetLanePosiontAndDircetions(const char* sectionID, const char* laneID) const;
		const vector<TurningAttr> GetLaneTurningAttributes(const char* sectionID, const char* laneID) const;
		const NearestDistance GetNeartestRoadandLaneAttributes(const char* sectionID, const char* laneID, const double &s, const double &t) const;
		const NearestRoadMark GetNeartestLaneRoadMark(const char* sectionID, const char* laneID) const;
		OdrInfo GetNearest(const double & longitude, const double &latitude,double &distance);
		bool  IsNearestRoad(const double & x, const double &y, const double &distance);
		vector<GeoCoordinatePoint> GetDataFromLine(const double &s, const double &sReal, const double &x, const double &y, const double &hdg, const double &length);
		vector<GeoCoordinatePoint> GetDataFromArc(const double &s, const double &sReal, const double &x, const double &y, const double &hdg, const double &length, const double &curvature);
		vector<GeoCoordinatePoint> GetDataFromSprial(const double &s, const double &sReal, const double &x0, const double &y0, const double &hdg, const double &length, const double &curvEnd, const double &curvStart, double &lastcurv);
		vector<GeoCoordinatePoint> GetDataFromPoly3(const double &s, const double &sReal, const double &x, const double &y, const double &hdg, const double &length, const double &a, const double &b, const double &c, const double &d);
		const glm::dvec2 GetRoadInitPos() const;
		const double GetLaneSpeed(const char* sectionID, const char* laneID) const;
		const vector<SignalPos>* GetLightPos() const;
		const vector<SignalPos>* GetStopLinePos() const;
		const glm::dvec2 GetRoadGeoCoordinateXY(const double &s, const double &t)const;
		const glm::dvec3 GetRoadGeoCoordinateXYHdg(const double &s, const double &t)const;
		const glm::dvec2 GetSTCoordinateFromRoad(const double &x, const double &y)const;
		const vector<PosXY>* GetLeftLinePosFromLane(const char* sectionID, const char* laneID) const;
		const vector<PosXY>* GetRightLinePosFromLane(const char* sectionID, const char* laneID) const;
		string GetLastSection()const;
		string GetPreSection(const char* sectionID)const;
		string GetSucSection(const char* sectionID)const;
		const bool IsFirstSection(const char* sectionID)const;
		const bool IsLastSection(const char* sectionID)const;
		RoadDrivingAttr GetRoadDriveTurningAttr();
		vector<GeoCoordinatePoint>* GetLeftBoundartData();
		vector<GeoCoordinatePoint>* GetRightBoundartData();
		vector<GeoCoordinatePoint>* GetReferenceData();
		vector<Object*>* GetObjectData();
		void GenerateLaneShp();
		void GenerateObjShp();
		void GenerateSingalObjShp();
		void part(string _str, vector<int> &_num, vector<char> &_op);
		string GetJunction();
		~Road();
	private:
		vector<LaneSection*>  *_vecLanSections;
		map<string, Signal*>  *_mapSignals;
		string _name;
		string _id;
		string _junction;
		double _length;
		vector<LaneOffset*> *_vecLaneOff;
		vector<Geometry*> *_vecData;
		vector<Elevation*> *_vecElev;
		RoadLink *_preLink;
		RoadLink *_sucLink;
		vector<Polygon*> _vecPolygon;
		vector<GeoCoordinatePoint> *_vecReferenceData;
		vector<GeoCoordinatePoint> *_vecLeftBoundaryData; //_laneBoundaryData
		vector<GeoCoordinatePoint> *_vecRightBoundaryData; //_laneBoundaryData
		glm::dvec2 _midPoint;
		glm::dvec2 _initPoint;
		glm::dvec2 _termPoint;
		glm::dvec2 _min;
		glm::dvec2 _max;
		vector<string> _sectionID;
		vector<Object*>* _vecObjs;

	};
}


