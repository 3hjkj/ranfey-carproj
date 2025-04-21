#pragma once
namespace OpenDrive
{
	class LaneWidth
	{
	public:
		LaneWidth();
		void SetAValue(const double & value);
		const double GetAValue() const;
		void SetBValue(const double & value);
		const double GetBBValue() const;
		void SetCValue(const double & value);
		const double GetCCValue() const;
		void SetDValue(const double & value);
		const double GetDValue() const;
		void SetOffSetValue(const double & value);
		const double GetOffSetValue() const;
		LaneWidth& operator=(const LaneWidth &width);
		LaneWidth(const LaneWidth &width);
		~LaneWidth();
	private:
		double _a = 0.0;
		double _b = 0;
		double _c = 0;
		double _d = 0;
		double _sOffset = 0;
	};
}

