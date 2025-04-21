#pragma once
#include"vector"
#include"glm/glm.hpp"
#include"DataDefines.h"
#include"Function.h"
using namespace std;
namespace OpenDrive
{
	class RepeatObj
	{
	public:
		RepeatObj();
		RepeatObj(const RepeatObj & obj);
		RepeatObj& operator =(const RepeatObj & obj);
		~RepeatObj();
		void SetDistanceValue(double &value);
		const double GetDistanceValue() const;
		void SetLengthValue(double &value);
		const double GetLengthValue() const;
		void SetWidthValue(double &value);
		const double GetWidthValue() const;
		void SetHeightValue(double &value);
		const double GetHeightValue() const;

	private:
		double _distance = 0.0;
		double _length;
		double _width;
		double _height;
	};
}

