#pragma once
#include <math.h>
#include"glm/glm.hpp"
namespace OpenDrive
{
	class NavinfoGISCoord
	{
	public:
		static NavinfoGISCoord* GetInstance();
		glm::dvec2 LatLonToUTM(const double &lon, const double &lat, const char* name);
		glm::dvec2 XYToLatLonUTM(double x, double y, bool southhemi, const char* name);

		double UTMCentralMeridian(const double &zone);
		double UTMCentral6Meridian(const double &zone);
		glm::dvec2 MapLatLonToXY(const double &phi, const double &lambda, const double &lambda0);
		glm::dvec2 MapXYToLatLon(double x, double y, double lambda0);
		double FootpointLatitude(double y);

		double ArcLengthOfMeridian(const double &phi);
	private:
		NavinfoGISCoord();
		~NavinfoGISCoord();
	private:
		const double pi = M_PI;

		const double sm_a = 6378137.0;
		const double sm_b = 6356752.3142;
		const double TMScaleFactor = 1.0;

	private:
		static NavinfoGISCoord *_instance;
	};
}



