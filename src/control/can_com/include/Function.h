#pragma once
#include"vector"
#include"cstring"
#include"string"
#include"DataDefines.h"
#include"Elevation.h"
#include"LaneOffset.h"
using namespace std;
namespace OpenDrive
{
	void QuickSort(vector<int> &vecData);
	void QuickSort(vector<float> &vecData);
	void QuickSort(vector<glm::dvec3> &vecData);//��������;
	void QuickSort(vector<int> &vecData, int flag);//flag 0, ��С����1���Ӵ�С��
	void QuickSort(vector<int> &vecData, vector<string> &vecLaneID);
	void QuickSort(vector<double> &vecData, vector<string> &vecSection);
	string DoubleToString(const double &value);
	bool GetComClockWise(const glm::dvec2 &p0, const glm::dvec2 &p1, const glm::dvec2 &p2);
	//bool GetComClockWise(const glm::dvec3 &p0, const glm::dvec3 &p1, const glm::dvec3 &p2);
	double GetLengthFromTwoPoints(const glm::dvec2 &p0, const glm::dvec2 &p1);
	string IntToString(const long int &value);
	void CalElevation(vector<GeoCoordinatePoint> &vecGeoCoor, const vector<Elevation*> *vecEle);
	void CalLaneOffset(vector<GeoCoordinatePoint> &vecGeoCoor, const vector<LaneOffset*> *vecEle);
	void CalElevation(GeoCoordinatePoint &geoCoor, const vector<Elevation*> *vecEle);
	void CalLaneOffset(GeoCoordinatePoint &geoCoor, const vector<LaneOffset*> *vecEle);
	double stringToDouble(string num);
	vector<string> split(const string& s, const string& sep);
	glm::dvec2 CalJointBetween(const glm::dvec2 &p0, const double &t0, const glm::dvec2 &p1, const double &t1);
	double GetDirectionRad(const glm::dvec2 &vline1);
	//bool IsInSidePolygon(const glm::vec2 &point, const vector<glm::dvec2> &vecData);
}

