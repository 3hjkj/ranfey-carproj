#ifndef _READ_CONFIG_
#define _READ_CONFIG_
#include <iostream>
#include <string.h>
#include <map>
#include <fstream>
#define COMMENT_CHAR '#'
// #include "main.h"
using namespace std;
namespace perception
{

    class ReadConfigCommon
    {
    private:
        /* data */
    public:
        ReadConfigCommon(/* args */);
        ~ReadConfigCommon();
        bool IsSpace(char c);
        bool IsCommentChar(char c);
        void Trim(string &str);
        bool AnalyseLine(const string &line, string &key, string &value);
        bool ReadConfig(const string &filename, map<string, string> &m);
        void PrintConfig(const map<string, string> &m);
    };
}
#endif