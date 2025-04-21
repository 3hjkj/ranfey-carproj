#pragma once
#include"DataDefines.h"
namespace OpenDrive
{
	class LaneRoadMark
	{
	public:
		LaneRoadMark();
		void SetColor(const char* color);
		const EnRoadMarkColor GetColor() const;
		void SetType(const char* type);
		const EnRoadMarkType GetType() const;
		void SetWeight(const char* weight);
		const EnRoadMarkWeight GetWeight() const;
		void SetOffSet(const double & offset);
		const double GetOffSet()const;
		LaneRoadMark(const LaneRoadMark & roadMark);
		LaneRoadMark& operator = (const LaneRoadMark & roadMark);
		~LaneRoadMark();
	private:
		EnRoadMarkColor _color;
		EnRoadMarkType _type;// = "broken" 
		EnRoadMarkWeight _weight;// = "standard"
		double _sOffset;
	};
}








