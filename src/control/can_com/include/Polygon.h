#pragma once
#include"string"
#include"vector"
#include"glm/glm.hpp"
using namespace std;
namespace OpenDrive
{
	class Polygon
	{
	public:
		Polygon();
		Polygon(const int &strSID, const int & strLaneID);
		Polygon(const Polygon &plygon);
		Polygon &operator = (const Polygon &plygon);
		void SetData(glm::vec2 value, int indx);
		void SetSID(int inSID);
		void SetLaneID(int inLaneID);
		void CalCenter();
		void CalLineCenter();
		void CalTriangleCenter();
		void CalDirection();
		void CalLineDirection();
		void CalTriangleDirection();
		const glm::vec2 & GetCenter();
		const glm::vec2 & GetDirection();
		const int &GetSectionID();
		const int &GetLaneID();
		const bool &GetEnterIsInSideFlag();
		const bool &GetExitIsInSideFlag();
		bool BEnterIsInSide(const glm::vec2 &intiPoint);
		bool BExitIsInSide(const glm::vec2 &termiPoint);
		bool BIsInSideLine(const glm::vec2 &point);
		bool BIsInSideTriangle(const glm::vec2 &point);
		bool BIsLine();
		bool BIsTriangle();
		void SetbEnterValue(const bool &bValue);
		void SetbExitValue(const bool &bValue);
		bool IsInSidePolygon(const glm::vec2 &point);
		~Polygon();
	private:
		int _strSID;
		int _strLaneID;
		bool _bEnter;
		bool _bExit;
		vector<glm::vec2> _vecData;
		glm::vec2 _center;
		glm::vec2 _direction;
	public:
		string _strEnter;
		string _strExit;
		string _strSectionName;
	};
}

