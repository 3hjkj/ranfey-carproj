#pragma once
#include"string"
using namespace std;
namespace OpenDrive
{
	class RoadLink
	{
	public:
		RoadLink();
		RoadLink& operator = (const RoadLink &link);
		RoadLink(const RoadLink &link);
		void SetContactPoint(const char* contactPoint);
		void SetElementId(const char* elementId);
		void SetElementType(const char* elementType);

		const char* GetContactPoint()const;
		const char* GetElementId()const;
		const char* GetElementType()const;

		~RoadLink();
	private:
		string _contactPoint;
		string _elementId; //= "19046"
		string _elementType;// = "junction" / >
	};
}
