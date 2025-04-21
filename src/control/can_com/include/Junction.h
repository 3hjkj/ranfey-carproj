#pragma once
#include"cstring"
#include"string"
#include"Connection.h"
#include"map"
#include"vector"
using namespace std;
namespace OpenDrive
{
	class Junction
	{
	public:
		Junction();
		Junction(const Junction &junc);
		Junction& operator =(const Junction &junc);
		void SetID(const char* id);
		const char* GetID() const;
		
		void SetName(const char* name);
		const char* GetName() const;
		void PushbackConnection(Connection *conn);
		const vector<Connection*> GetAllConnection() const;
		const vector<string> GetConnectingRoad(const char* roadID) const;
		~Junction();
	private:
		string _name;
		string _id;
		map<string, Connection*> *_mapConn;
	};
}
