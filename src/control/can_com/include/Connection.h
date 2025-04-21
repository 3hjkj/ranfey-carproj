#pragma once
#include"cstring"
#include"string"
#include"unordered_map"
using namespace std;
namespace OpenDrive
{
	class Connection
	{
	public:
		Connection();
		Connection& operator= (const Connection &conn);
		Connection(const Connection &conn);
		void SetID(const char* id);
		const char* GetID() const;
		void SetIncomingRoad(const char* incomingRoad);
		const char* GetIncomingRoad() const;
		void SetConnectingRoad(const char* incomingRoad);
		const char* GetConnectingRoad() const;
		void SetContactPoint(const char* contactPoint);
		const char* GetContactPoint() const;
		void SetLaneLinkFromTo(const char* laneLinkFrom, const char* laneLinkTo);
		string GetLaneLinkFrom(const char* laneLinkTo) const;
		string GetLaneLinkTo(const char* laneLinkFrom) const;

		~Connection();

	private:

		string _id;
		string _incomingRoad;
		string _connectingRoad;
		string _contactPoint;
		unordered_map<string, string> *_laneLinkFromTo;
	};
}
