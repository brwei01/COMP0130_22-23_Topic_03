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
        Object(vector<double> vdetect_parameter_, int ndetect_class_);
        ~Object();

        public:
        enum classname
        {
            person = 3,
            car = 1
        };

        public:
        vector<double> vdetect_parameter;
        int ndetect_class;
        bool bdynamic_;

        public:
        vector<double> GetDetectParameter();
        int GetDetectClass();      
    };

    class Object3D
    {
    public:
        Object3D();
        Object3D(const int& nframe_id_, const vector<double>& vbbox_2d_, const vector<double>& vbbox_birdview_, const string& sdetect_class_);
        ~Object3D();

    public:
        int nframe_id;
        vector<double> vbbox_2d;
        vector<double> vbbox_birdview;
        string sdetect_class;

    public:
        int GetFrameID();
        vector<double> Get2dBbox();
        vector<double> GetBboxBV();
        string GetDetectClassBV();
    };

}

#endif