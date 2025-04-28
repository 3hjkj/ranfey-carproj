#include <vector>
#include <iostream>
#include <string>
#include <math.h>
#include <fstream>
#include "dbscan.h"
using namespace std;
namespace perception
{
    class min_rotate_rect
    {
    private:
        /* data */
        double EPS = 1e-10;
        // x_min, x_max,y_min,y_max
        double area = 1000;
        // vector<double> final_points; // x_min, x_max,y_min,y_max,rad
        int counter = 0;

    public:
        min_rotate_rect(/* args */);
        ~min_rotate_rect();
        vector<double> GetRad(const vector<points> &data_in);
        vector<double> MinRotateRect(const vector<points> &data_in);
        vector<points> RotateRect(const vector<points> &data_in,
                                  double &rad);
        vector<double> GetArea(const vector<points> &data_in, double &rad);
        vector<points> GetFourPoints(const vector<double> final_points);
        vector<double> GetFinalShape(vector<points> &data_in);
    };
}