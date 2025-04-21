#pragma once
#include"string"
#include"vector"
#include"SignalReference.h"
using namespace std;
namespace OpenDrive
{
	class Signal
	{
	public:
		Signal();
		Signal &operator = (const Signal &sigl);
		Signal(const Signal &sigl);
		void SetID(const char* id);
		const char* GetID() const;
		void SetName(const char* name);
		const char* GetName() const;

		void SetCoodinateS(const double& value);
		const double GetCoodinateS() const;

		void SetCoodinateT(const double& value);
		const double GetCoodinateT() const;

		void SetCoodinatezOffset(const double& value);
		const double GetCoodinatezOffset() const;
		void PushSignal(SignalReference *siglRef);

		~Signal();
	private:
		double _s;
		double _t;
		string _id;
		string _name;
		double _zOffset;
		vector<SignalReference*> *_vecSigRef;

	};
}

