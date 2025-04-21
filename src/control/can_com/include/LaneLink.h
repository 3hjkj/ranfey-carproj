#pragma once
#include"string"
using namespace std;
namespace OpenDrive
{
	class LaneLink
	{
	public:
		LaneLink();
		LaneLink(const LaneLink&link);
		void SetPreID(const char* id);
		void SetSucID(const char* id);
		const string GetPreLaneID()const;
		const string GetSucLaneID()const;
		LaneLink& operator = (const LaneLink&link);
		~LaneLink();
	private:
		string _preID ;
		string _sucID;
		//<predecessor id = "-2" / >
		//	<successor id = "-3" / >
	};
}

