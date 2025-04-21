#pragma once
namespace OpenDrive
{
	class Elevation
	{
	public:
		Elevation();
		void SetAValue(const double & value);
		const double GetAValue() const;
		void SetBValue(const double & value);
		const double GetBBValue() const;
		void SetCValue(const double & value);
		const double GetCCValue() const;
		void SetDValue(const double & value);
		const double GetDValue() const;
		void SetSValue(const double & value);
		const double GetSValue() const;
		Elevation& operator=(const Elevation &elev);
		Elevation(const Elevation &elev);
		~Elevation();
	private:
		double _a = 0.0;
		double _b = 0;
		double _c = 0;
		double _d = 0;
		double _s = 0;
	};
}
