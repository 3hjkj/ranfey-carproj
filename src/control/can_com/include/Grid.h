#pragma once
#include"cstring"
#include"string"
#include"set"
using namespace std;
namespace OpenDrive
{
	class Grid
	{
	public:
		Grid();
		Grid& operator= (const Grid &grid);
		Grid(const Grid &grid);
		void SetRoadID(const char* roadID);
		set<string> GetRoadID();
		~Grid();
	private:
		set<string> _vecRoad;
	};

}

