#pragma once
#include"DataDefines.h"
#include"vector"
#include"odrSpiral.h"
#include"glm/glm.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include"Function.h"
using namespace std;
namespace OpenDrive
{
	class Geometry
	{
	public:
		Geometry();
		Geometry(const Geometry &geo);
		Geometry & operator = (const Geometry &geo);
		double GetLaneDistanceFromLine(const double & longitude, const double &latitude, const double &slen);
		double GetLaneDistanceFromArc(const double & longitude, const double &latitude, const double &slen);
		double GetLaneDistanceFromSpiral(const double & longitude, const double &latitude, const double &slen);
	
		bool GetNearestInfoFromLine(const double & longitude, const double &latitude,double &distance, glm::dvec2 & vec);
		bool GetNearestInfoFromArc(const double & longitude, const double &latitude, double &distance, glm::dvec2 & vec);
		bool GetNearestInfoFromSprial(const double & longitude, const double &latitude, double &distance, glm::dvec2 & vec);
		bool GetNearestInfoFromPoly3(const double & longitude, const double &latitude, double &distance, glm::dvec2 & vec);


		bool IsNearestInfoFromLine(const double &x, const double &y, const double &distance);
		bool IsNearestInfoFromArc(const double &x, const double &y, const double &distance);
		bool IsNearestInfoFromSprial(const double &x, const double &y, const double &distance);
		bool IsNearestInfoFromPoly3(const double &x, const double &y, const double &distance);

		const glm::dvec2 GetRoadGeoCoordinateXYFromLine(const double &s, const double &t)const;
		const glm::dvec2 GetRoadGeoCoordinateXYFromArc(const double &s, const double &t)const;
		const glm::dvec2 GetRoadGeoCoordinateXYFromSpiral(const double &s, const double &t)const;
		const glm::dvec2 GetRoadGeoCoordinateXYFromPloy3(const double &s, const double &t)const;

		const glm::dvec3 GetRoadGeoCoordinateXYHdgFromLine(const double &s, const double &t)const;
		const glm::dvec3 GetRoadGeoCoordinateXYHdgFromArc(const double &s, const double &t)const;
		const glm::dvec3 GetRoadGeoCoordinateXYHdgFromSpiral(const double &s, const double &t)const;
		const glm::dvec3 GetRoadGeoCoordinateXYHdgFromPloy3(const double &s, const double &t)const;
		
		const bool GetRoadGeoCoordinateSTFromLine(const glm::dvec3 vec, glm::dvec2 &st, double &dis)const;
		const bool GetRoadGeoCoordinateSTFromArc(const glm::dvec3 vec, glm::dvec2 &st, double &dis)const;
		const bool GetRoadGeoCoordinateSTFromSpiral(const glm::dvec3 vec, glm::dvec2 &st, double &dis)const;
		const bool GetRoadGeoCoordinateSTFrompPloy3(const glm::dvec3 vec, glm::dvec2 &st, double &dis)const;

		vector<glm::dvec3> GetDataFromLine();
		vector<glm::dvec3> GetDataFromArc();
		vector<glm::dvec3> GetDataFromSpiral();
		//bool GetComClockWise(const glm::dvec2 &p0, const glm::dvec2 &p1, const glm::dvec2 &p2);
		const vector<Position> GetLanePosiontAndDircetionsFromLine(const double & initPosition, const double &lastPosition, double &position)const;
		const vector<Position> GetLanePosiontAndDircetionsFromArc(const double & initPosition, const double &lastPosition, double &position)const;
		const vector<Position> GetLanePosiontAndDircetionsFromSpiral(const double & initPosition, const double &lastPosition, double &position)const;
		//const vector<Position> GetLanePosiontAndDircetionsFromPloy3(const double & initPosition, const double &lastPosition, double &position)const;

		const vector<TurningAttr> GetLaneTurningAttributesFromLine(const double & initPosition, const double &lastPosition, double &position,const int &flag)const;
		const vector<TurningAttr> GetLaneTurningAttributesFromArc(const double & initPosition, const double &lastPosition, double &position, const int &flag)const;
		const vector<TurningAttr> GetLaneTurningAttributesFromSpiral(const double & initPosition, const double &lastPosition, double &position, const int &flag)const;
		//const vector<TurningAttr> GetLaneTurningAttributesFromPloy3(const double & initPosition, const double &lastPosition, double &position, const int &flag)const;
		~Geometry();
	public:
		double _hdg;
		double _length;
		double _s;
		double _x;
		double _y;
		EnGeometryType _flag;
		double _curvStart;
		double _curvEnd;
		double _curvature;
		double _a = 0.0;
		double _b = 0.0;
		double _c = 0.0;
		double _d = 0.0;
	};
}

