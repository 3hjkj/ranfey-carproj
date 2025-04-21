#pragma once

namespace OpenDrive
{
	class Speed
	{
	public:
		Speed();
		Speed& operator = (const Speed &speed);
		Speed(const Speed &speed);
		void SetsOffset(const double &value);
		const double GetsOffset() const;
		void SetMax(const double &value);
		const double GetMax() const;
		~Speed();
	private:
		double _sOffset;
		double _max;
	};
}

