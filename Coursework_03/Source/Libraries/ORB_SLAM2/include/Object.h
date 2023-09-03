#ifndef MYSLAM_OBJECT_H
#define MYSLAM_OBJECT_H

#include <fstream>
#include <mutex>
#include <set>
#include "MapPoint.h"

using namespace std;

namespace ORB_SLAM2
{
    class MapPoint;
    class Object
    {
        public:
        Object();
        Object(vector<double> vdetect_parameter_, 
                string sdetect_class_,
                vector<vector<int>> vmask_coords);
        ~Object();

        public:
        vector<double> vdetect_parameter;
        string sdetect_class;
        vector<vector<int>> vmask_coords;

        public:
        vector<double> GetDetectParameter();
        string GetDetectClass();
        vector<vector<int>> GetMaskCoords();      
    };
}

#endif