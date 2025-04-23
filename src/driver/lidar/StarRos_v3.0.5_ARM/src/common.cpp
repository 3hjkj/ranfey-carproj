#include <rclcpp/rclcpp.hpp>

#include"ioapi.h"
#include"rfans_driver.h"
#include <unistd.h>
#include <string>
#include <sstream>
#include <iostream>
#include <thread>
#include <stdlib.h>
#include<cstdlib>
#include<cmath>
#include <Eigen/Eigen>
#include "common.h"
#include"filter.h"
using namespace Eigen;
using namespace std;

void file_time_patch(string &time_str)
{
  ulong time_deci_cnt=time_str.size()-time_str.find(".")-1;
  switch (time_deci_cnt) {
  case 4:
    time_str.insert(time_str.find(".")+1,"00");
    break;
  case 5:
    time_str.insert(time_str.find(".")+1,"0");
    break;
  default:
    break;
  }
}

vector<crdFilterPara_S> getFilterXYZ(string filter_path)
{
    ifstream file_;
    file_.open(filter_path);
    string line_str;
    vector<crdFilterPara_S> filterXYZV;
    if(file_.is_open())
    {
      while (!file_.eof())
      {
        getline(file_,line_str);
        crdFilterPara_S _filterXYZ;
        vector<float> valueV ;
        memset(&_filterXYZ,0,sizeof(crdFilterPara_S));
        char *p=strtok(const_cast<char *>(line_str.c_str()),",");
        while(p!=NULL)
        {
          string p_str(p);
          file_time_patch(p_str);
          valueV.push_back(stof(p_str));
          p=strtok(NULL,",");
        }
        if(valueV.size()>0)
        {
          memcpy(&_filterXYZ,valueV.data(),sizeof(float)*valueV.size());
          filterXYZV.push_back(_filterXYZ);
        }
      }
    }
    else {
      std::cout <<"file read failed!"<<std::endl;
    }
    return filterXYZV;
}
float getTemperFrStr(string  filePath_)
{

    int count_ = 0;
    int degin_ = -1;
    while ((degin_ = filePath_.find("-", degin_ + 1)) != string::npos)
    {
        count_++;
        degin_ += 1;
    }

    if (count_ == 2)
    {
      return  -999;
    }
    else if (count_ == 3)
    {
        int idx_s = filePath_.find_last_of("-");
        int idx_e = filePath_.find_last_of(".");
        string temp_ = filePath_.substr(idx_s + 1, idx_e - idx_s - 1);
        return atof(temp_.c_str());
    }

}
string getNameFromPath(string input_path,string& out_path )
{
    int dx_=input_path.find_last_of("/");
    string name=input_path.substr(dx_+1);
    out_path=input_path.substr(0,dx_);
    return name;
}
