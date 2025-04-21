#pragma once
#include"string"
using namespace std;
namespace OpenDrive
{
	class UserData
	{
	public:
		UserData();
		UserData& operator = (const UserData &usData);
		UserData(const UserData &usData);


		void SetCode(const char* id);
		const string GetCode() const;

		void SetsOffset(const char* sOffset);
		const string GetsOffset() const;

		void SetLaneStyle(const char* laneStyle);
		const string GetLaneStyle() const;

		void SetMapping(const char* mapping);
		const string GetMapping() const;

		void SetPos(const char* pos);
		const string GetPos() const;

		void SetStyle(const char* style);
		const string GetStyle() const;

		void SetId(const char* id);
		const string GetId() const;


		~UserData();
	private:
		string _code;
		string _sOffset;
		string _laneStyle;
		string _mapping;
		string _pos;
		string _style;
		string _id;

	};
}

