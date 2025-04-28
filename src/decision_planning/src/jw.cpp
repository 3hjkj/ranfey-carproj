#include<iostream>
#include<string>
#include<math.h>
using namespace std;
#define PI 3.1414926
string ECEFtoWGS84(double x, double y, double z){
		double a, b, c, d;
		double Longitude;// 经度
		double Latitude;// 纬度
		double Altitude;// 海拔高度
		double p, q;
		double N;
		a = 6378137.0;
		b = 6356752.31424518;
		c = sqrt(((a * a) - (b * b)) / (a * a));
		d = sqrt(((a * a) - (b * b)) / (b * b));
		p = sqrt((x * x) + (y * y));
		q = atan2((z * a), (p * b));
		Longitude = atan2(y, x);
		Latitude = atan2((z + (d * d) * b * pow(sin(q), 3)),
				(p - (c * c) * a * pow(cos(q), 3)));
		N = a / sqrt(1 - ((c * c) * pow(sin(Latitude), 2)));
		Altitude = (p / cos(Latitude)) - N;
		Longitude = Longitude * 180.0 /PI;
		Latitude = Latitude * 180.0 / PI;
		cout<<"Longitude="<<Longitude<<endl;
        cout<<"Latitude="<<Latitude<<endl;
        return 0;
}
int main()
{
    while (true)
    {
        ECEFtoWGS84(1805.497002,-679.706474,260.99);
    }
    return 0;
}