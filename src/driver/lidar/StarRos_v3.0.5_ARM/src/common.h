/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#ifndef __COMMON_H
#define __COMMON_H
#include <string>
#include <string.h>
#include <vector>
#include"filter.h"
using namespace std;
vector<crdFilterPara_S> getFilterXYZ(string filter_path);
float getTemperFrStr(string  filePath_);
string getNameFromPath(string input_path,string& out_path );


#endif //__RFANS_IOAPI_H
