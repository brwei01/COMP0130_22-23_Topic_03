#include "Object.h"
#include "vector"
#include "math.h"
#define PI 3.1415926535

namespace ORB_SLAM2 
{
    Object::Object()
    {
        bdynamic_ = false;
        vdetect_parameter = {0,0,0,0};
        ndetect_class = -1;
    }

    Object::Object(vector<double> vdetect_parameter_, int ndetect_class_):
    vdetect_parameter(vdetect_parameter_), ndetect_class(ndetect_class_){}

    Object::~Object(){}

    vector<double> Object::GetDetectParameter()
    {
        return vdetect_parameter;
    }

    int Object::GetDetectClass()
    {
        return ndetect_class;
    }


    // CLASS OBJECT 3D
    Object3D::Object3D()
    {
        nframe_id = 0;
        vbbox_2d = {0,0,0,0};
        vbbox_birdview = {0,0,0,0};
        sdetect_class = "DoNotCare";
    }

    Object3D::Object3D( const int& nframe_id_,
                    const std::vector<double>& vbbox_2d_,
                    const std::vector<double>& vbbox_birdview_,
                    const std::string& sdetect_class_):
    nframe_id(nframe_id_), vbbox_2d(vbbox_2d_), vbbox_birdview(vbbox_birdview_), sdetect_class(sdetect_class_){}

    Object3D::~Object3D(){}

    int Object3D::GetFrameID()
    {
        return nframe_id;
    }

    vector<double> Object3D::Get2dBbox()
    {
        return vbbox_2d;
    }

    vector<double> Object3D::GetBboxBV()
    {
        return vbbox_birdview;
    }

    string Object3D::GetDetectClassBV()
    {
        return sdetect_class;
    }


}