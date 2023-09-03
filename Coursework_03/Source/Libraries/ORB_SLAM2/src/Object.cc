#include "Object.h"
#include "vector"
#include "math.h"
#define PI 3.1415926535

namespace ORB_SLAM2 
{
    Object::Object()
    {
        vdetect_parameter = {0,0,0,0};
        sdetect_class = "None";
        vmask_coords = {{0,0}};
    }

    Object::Object(vector<double> vdetect_parameter_, string sdetect_class_, vector<vector<int>> vmask_coords_):
    vdetect_parameter(vdetect_parameter_), sdetect_class(sdetect_class_), vmask_coords(vmask_coords_){}

    Object::~Object(){}

    vector<double> Object::GetDetectParameter()
    {
        return vdetect_parameter;
    }

    string Object::GetDetectClass()
    {
        return sdetect_class;
    }

    vector<vector<int>> Object::GetMaskCoords()
    {
        return vmask_coords;
    }
}