#pragma once

namespace OpenDrive
{
	//<height sOffset = "0" inner = "0.2" outer = "0.2" / >
	class Height
	{
	public:
		Height();
		Height& operator = (const Height &height);
		Height(const Height &height);
		void SetsOffset(const double &value);
		const double GetsOffset() const;
		void SetInner(const double &value);
		const double GetInner() const;
		void SetOuter(const double &value);
		const double GetOuter() const;
		~Height();
	private:
		double _sOffset;
		double _inner;
		double _outer;
	};
}


