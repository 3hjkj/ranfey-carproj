#pragma once
#include"string"
using namespace std;
namespace OpenDrive
{
	class SignalReference
	{
	public:
		SignalReference();

		SignalReference& operator=(const SignalReference &siglRef);
		SignalReference (const SignalReference &siglRef);

		void SetID(const char* id);
		const char* GetID() const;

		void SetCoodinateS(const double& value);
		const double GetCoodinateS() const;

		void SetCoodinateT(const double& value);
		const double GetCoodinateT() const;

		~SignalReference();
	private:
		double _s = 0.0;
		double _t = 0.0;
		string _id;
	};
}

