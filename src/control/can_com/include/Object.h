#pragma once
#include"vector"
#include"glm/glm.hpp"
#include"DataDefines.h"
#include"Function.h"
#include"RepeatObj.h"
using namespace std;
namespace OpenDrive
{
	class Object
	{
	public:
		Object();
		Object(const Object & obj3d);
		Object& operator =(const Object & obj);
		void SetName(const char* name);
		const char* GetName() const;
		void SetType(const char* type);
		const char* GetType() const;
		void SetSValue(double &value);
		const double GetSValue() const;
		void SetTValue(double &value);
		const double GetTValue() const;
		void PushbackData(glm::dvec3 vec);
		vector<glm::dvec3>* GetCoords();


		void SetWidthValue(double &value);
		const double GetWidthValue() const;
		void SetHeightValue(double &value);
		const double GetHeightValue() const;
		//zOffset
		void SetzOffsetValue(double &value);
		const double GetzOffsetValue() const;
		void SetRepeatObjValue(RepeatObj *repeatObj);
		const RepeatObj * GetRepeatObjValue() const;

		~Object();
	private:

		string _name;
		string _type;
		double _s;
		double _t;
		double _width;
		double _height;
		double _zOffset;
		vector<glm::dvec3> *_vecCoords;
		RepeatObj *_repeatObj;

		//<repeat s = "2.73406954883456" length = "79.566455604047" distance = "15" tStart = "-13.3226013033395" tEnd = "-13.3226013033395" widthStart = "5.7990000000000004e+00" widthEnd = "5.7990000000000004e+00" heightStart = "6.6230000000000002e+00" heightEnd = "6.6230000000000002e+00" zOffsetStart = "-3.8850000000000007e-01" zOffsetEnd = "-3.8850000000000007e-01" / >
	};
}

