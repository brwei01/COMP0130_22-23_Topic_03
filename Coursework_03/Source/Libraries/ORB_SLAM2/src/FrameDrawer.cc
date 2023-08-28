/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
 * of Zaragoza) For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#include "FrameDrawer.h"
#include "Tracking.h"

#include "Object.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mutex>

using namespace ::std;

namespace ORB_SLAM2 {

FrameDrawer::FrameDrawer(Map *pMap) : mpMap(pMap) {
  mState = Tracking::SYSTEM_NOT_READY;
  mIm = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
}

cv::Mat FrameDrawer::DrawFrame() {
  cv::Mat im;
  vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
  vector<int>
      vMatches; // Initialization: correspondeces with reference keypoints
  //vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
  // ************************************
  vector<KeyPointsWithInfo> vCurrentKeys;
  // ************************************
  vector<bool> vbVO, vbMap;          // Tracked MapPoints in current frame
  int state;                         // Tracking state

  // Copy variables within scoped mutex
  {
    unique_lock<mutex> lock(mMutex);
    state = mState;
    if (mState == Tracking::SYSTEM_NOT_READY)
      mState = Tracking::NO_IMAGES_YET;

    mIm.copyTo(im);

    if (mState == Tracking::NOT_INITIALIZED) {
      vCurrentKeys = mvCurrentKeys;
      vIniKeys = mvIniKeys;
      vMatches = mvIniMatches;
    } else if (mState == Tracking::OK) {
      vCurrentKeys = mvCurrentKeys;
      vbVO = mvbVO;
      vbMap = mvbMap;
    } else if (mState == Tracking::LOST) {
      vCurrentKeys = mvCurrentKeys;
    }
  } // destroy scoped mutex -> release mutex

  if (im.channels() < 3) // this should be always true
    cvtColor(im, im, cv::COLOR_GRAY2BGR);


  // Draw
  if (state == Tracking::NOT_INITIALIZED) // INITIALIZING
  {
    for (unsigned int i = 0; i < vMatches.size(); i++) {
      if (vMatches[i] >= 0) {
        cv::line(im, vIniKeys[i].pt, vCurrentKeys[vMatches[i]].keypoints[0].pt,
                 cv::Scalar(0, 255, 0));
      }
    }
  } else if (state == Tracking::OK) // TRACKING
  {
    mnTracked = 0;
    mnTrackedVO = 0;
    const float r = 5;
    const int n = vCurrentKeys.size();
    
    // *********************************
    // MODIFICATIONS: draw bounding box
    for (int k = 0; k < objects_curFD.size(); ++k)
    { 
      /*
      // THIS IS TO CHECK THE CONTENT OF 'objects_curFD'
      std::cout << "the object is: ";
      for (const auto& param : objects_curFD[k]->vdetect_parameter) {
          std::cout << param << " ";
      }
      std::cout << objects_curFD[k]->ndetect_class << std::endl;          
      // THIS CAN BE ANNOTATED
      */
      if (objects_curFD[k] -> sdetect_class == "Car" || objects_curFD[k] -> sdetect_class == "Cyclist")
      {
        // 3D BIRDVIEW BBOX ON MAP
        vector<double> box_BV = objects_curFD[k] -> vbbox_birdview;
        double left_BV = box_BV[0];
        double top_BV = box_BV[1];
        double right_BV = box_BV[2];
        double bottom_BV = box_BV[3];
        
        bool updated = false;
        // initialize distance
        float dist2cam = 999.0f;
        for (int i=0; i<n; i++)
        {

          // std::cout << "vCurrentKeys[" << i << "].pt = " << vCurrentKeys[i].pt << std::endl; 
          if (vbVO[i] || vbMap[i]) 
          {
            // std::cout << "vbVO[" << i << "] = " << vbVO[i] << ", vbMap[" << i << "] = " << vbMap[i] << std::endl;
            // TO CHECK THE mvCurrentKeys IS UPDATED:
            // float dist2cam0 = mvCurrentKeys[i].info;
            // std::cout << "****************dist to cam:" << dist2cam0 << std::endl;
            

            // 2D POINT ON FRAME
            cv::Point2f pt1, pt2;
            pt1.x = vCurrentKeys[i].keypoints[0].pt.x - r;
            pt1.y = vCurrentKeys[i].keypoints[0].pt.y - r;
            pt2.x = vCurrentKeys[i].keypoints[0].pt.x + r;
            pt2.y = vCurrentKeys[i].keypoints[0].pt.y + r;

            /*
            // This is a match to a MapPoint in the map
            if (vbMap[i]) {
              cv::rectangle(im, pt1, pt2, cv::Scalar(0, 255, 0));
              cv::circle(im, vCurrentKeys[i].pt, 1, cv::Scalar(0, 255, 0), -1);
              mnTracked++;
            } else // This is match to a "visual odometry" MapPoint created in the
                    // last frame
            {
              cv::rectangle(im, pt1, pt2, cv::Scalar(255, 0, 0));
              cv::circle(im, vCurrentKeys[i].pt, 2, cv::Scalar(255, 0, 0), -1);
              mnTrackedVO++;
            }
            */

            // *********************
            // MODIFICATIONS: make the selected dot red

            // 2D BBOX ON FRAME
            /*
            vector<double> box = objects_curFD[k]->vbbox_2d;
            double left = box[0];
            double top = box[1];
            double right = box[2];
            double bottom = box[3];

            float kp_u = vCurrentKeys[i].keypoints[0].pt.x;
            float kp_v = vCurrentKeys[i].keypoints[0].pt.y;
            */

            // 3Dp point coordinates 
            float PcX = vCurrentKeys[i].mapPointCoords[0];
            float PcY = vCurrentKeys[i].mapPointCoords[1];
            float PcZ = vCurrentKeys[i].mapPointCoords[2];

            std::cout << "***************** the birdview bbox: " << left_BV << top_BV << right_BV << bottom_BV << endl; 
            std::cout << "================= the point under camera coords: " << PcX << PcZ << endl;

            // if (kp_u > left + 2 && kp_u < right - 2 && kp_v > top + 2 && kp_v < bottom - 2)
            if (PcX > left_BV && PcX < right_BV && PcZ > top_BV && PcZ < bottom_BV)
            {
              /*
              // Get distance and convert to string
              float dist2cam = vCurrentKeys[i].info;
              std::string distStr = "Dist: " + std::to_string(dist2cam);
              */

              // update the dist to cam of this point
              if (vCurrentKeys[i].info < dist2cam && !std::isnan(vCurrentKeys[i].info) && vCurrentKeys[i].info>0) // and theres a minimum distance can be updated
              {
                dist2cam = vCurrentKeys[i].info;
                updated = true;
              }  
              // Draw points
              // std::cout << "dynamic point found!" << std::endl;
              cv::rectangle(im, pt1, pt2, cv::Scalar(0,0,200));
              cv::circle(im, vCurrentKeys[i].keypoints[0].pt, 1, cv::Scalar(0,0,200), -1);
              /*
              // Draw dist2cam text above the point
              cv::Point textPos(pt1.x, pt1.y - 10); // Adjust the position as needed
              cv::putText(im, distStr, textPos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 200), 1);
              */
              mnTracked++;
            }
            else
            {
              // std::cout<< "no dynamic point found" << std::endl;
              cv::rectangle(im, pt1, pt2, cv::Scalar(0,255,0));
              cv::circle(im, vCurrentKeys[i].keypoints[0].pt, 1, cv::Scalar(0,255,0),-1);
              mnTracked++;
            }
          }  
        } // end enumerate points

      

        if(updated){
                
          // draw bounding box
          cv::Point pt11, pt22;
          pt11 = cv::Point(objects_curFD[k]->vbbox_2d[0], objects_curFD[k]->vbbox_2d[1]);
          pt22 = cv::Point(objects_curFD[k]->vbbox_2d[2], objects_curFD[k]->vbbox_2d[3]);
          
          // Add text displaying dist2cam over the bounding box
          // std::cout << "car detected!" << pt11 << pt22 << std::endl;
          cv::rectangle(im, pt11, pt22, cv::Scalar(0,200,200));
          std::cout << "This is bounding box: " << pt11 << pt22 << std::endl;
          std::cout << "Corresponding birdview bbox: " << left_BV << ' '<< top_BV << ' ' << right_BV << ' ' << bottom_BV << endl; 
          std::cout << "min distance to camera: " << std::to_string(dist2cam) << std::endl;
          std::string labelText = "Distance: " + std::to_string(dist2cam);
          // show only 2 decimal places
          size_t decimalPos = labelText.find('.');
          if (decimalPos != std::string::npos && labelText.size() > decimalPos + 3) {
              labelText = labelText.substr(0, decimalPos + 3);
          }


          int font = cv::LINE_AA;
          double fontScale = 0.5;
          int thickness = 2;
          cv::Point textPosition(pt11.x, pt11.y - 5); // Adjust the position as needed
          cv::putText(im, labelText, textPosition, font, fontScale, cv::Scalar(0, 0, 255), thickness);

          // Check if dist2cam is smaller than 10 and add a "Caution" prompt
          if (dist2cam < 5) {
            std::string cautionText = "Caution";
            int cautionFont = cv::FONT_HERSHEY_SIMPLEX;
            double cautionFontScale = 0.5;
            int cautionThickness = 2;
            cv::Point cautionTextPosition(pt11.x, pt11.y - 20); // Adjust the position as needed
            cv::putText(im, cautionText, cautionTextPosition, cautionFont, cautionFontScale, cv::Scalar(0, 0, 255), cautionThickness);
          }    
        }


      } // if detection result is car
    } // enum all bounding boxes
  } // if tracking is ok, Draw

  // write in status info
  cv::Mat imWithInfo;
  DrawTextInfo(im, state, imWithInfo);

  return imWithInfo;
}

void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText) {
  stringstream s;
  if (nState == Tracking::NO_IMAGES_YET)
    s << " WAITING FOR IMAGES";
  else if (nState == Tracking::NOT_INITIALIZED)
    s << " TRYING TO INITIALIZE ";
  else if (nState == Tracking::OK) {
    if (!mbOnlyTracking)
      s << "SLAM MODE |  ";
    else
      s << "LOCALIZATION | ";
    int nKFs = mpMap->KeyFramesInMap();
    int nMPs = mpMap->MapPointsInMap();
    s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
    if (mnTrackedVO > 0)
      s << ", + VO matches: " << mnTrackedVO;
  } else if (nState == Tracking::LOST) {
    s << " TRACK LOST. TRYING TO RELOCALIZE ";
  } else if (nState == Tracking::SYSTEM_NOT_READY) {
    s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
  }

  int baseline = 0;
  cv::Size textSize =
      cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

  imText = cv::Mat(im.rows + textSize.height + 10, im.cols, im.type());
  im.copyTo(imText.rowRange(0, im.rows).colRange(0, im.cols));
  imText.rowRange(im.rows, imText.rows) =
      cv::Mat::zeros(textSize.height + 10, im.cols, im.type());
  cv::putText(imText, s.str(), cv::Point(5, imText.rows - 5),
              cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);
}

void FrameDrawer::Update(Tracking *pTracker) {
  unique_lock<mutex> lock(mMutex);


  //*****************************
  // MODIFICATIONS: added necessary variables
  objects_curFD = pTracker -> mCurrentFrame.objects_cur_;
  // vbInDynamic_mvKeys = pTracker -> mCurrentFrame.vbInDynamic_mvKeys; 
  // END MODIFICATION
  //*****************************


  pTracker->mImGray.copyTo(mIm);

  // mvCurrentKeys = pTracker->mCurrentFrame.mvKeys;
  // ***********************************************
  const std::vector<cv::KeyPoint>& cvKeys = pTracker->mCurrentFrame.mvKeys;
  mvCurrentKeys.clear();
  for (const cv::KeyPoint& cvKey:cvKeys){
    std::vector<cv::KeyPoint> keypointsWithInfo;
    keypointsWithInfo.push_back(cvKey); // push cvKey to the vector
    std::vector<float> ptCoords = {0.0f, 0.0f, 0.0f}; // Initialize pMP coords with zeros
    int extraInfo = -999.0f; // Default value if no valid MapPoint
    mvCurrentKeys.emplace_back(keypointsWithInfo, ptCoords, extraInfo);
  }
  
  N = mvCurrentKeys.size();
  mvbVO = vector<bool>(N, false);
  mvbMap = vector<bool>(N, false);
  mbOnlyTracking = pTracker->mbOnlyTracking;
  cv::Mat mRcw_curr = pTracker -> mCurrentFrame.GetRcw();
  cv::Mat mtcw_curr = pTracker -> mCurrentFrame.GetTcw();

  if (pTracker->mLastProcessedState == Tracking::NOT_INITIALIZED) {
    mvIniKeys = pTracker->mInitialFrame.mvKeys;
    mvIniMatches = pTracker->mvIniMatches;
  } else if (pTracker->mLastProcessedState == Tracking::OK) {
    for (int i = 0; i < N; i++) {
      MapPoint *pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
      if (pMP) {
        if (!pTracker->mCurrentFrame.mvbOutlier[i]) {
          // *********
          // calculate distance:
          cv::Mat Ow = pTracker->mCurrentFrame.GetCameraCenter(); 
          cv::Mat Pos = pMP->GetWorldPos(); 
          //std::cout << "Ow[" << i << "]: " << Ow << std::endl;
          //std::cout << "Pos[" << i << "]: " << Pos << std::endl;
          cv::Mat PC = Pos - Ow;
          const float dist2cam = cv::norm(PC); // is it needed to * dist median depth (or dist between first 2 frames) (unit) here? -- it doesnt seem to help

          // std::cout << "Dist["<< i <<"]: " << dist2cam << std::endl;
          // std::cout << "Before Update - info[" << i << "]: " << mvCurrentKeys[i].info << std::endl;
          mvCurrentKeys[i].info = dist2cam;
          //std::cout << "After Update - info[" << i << "]: " << mvCurrentKeys[i].info << std::endl;
          // **********
          // std::cout << "distance to camera" << dist2cam << std::endl;

          // 3D POINT BIRDVEW transfered from absolute coordinates to camera coordinate system
          
          
          cv::Mat P = pMP->GetWorldPos();  
          const cv::Mat Pc = mRcw_curr*P+mtcw_curr; 
          const float &PcX = Pc.at<float>(0);
          const float &PcY = Pc.at<float>(1);
          const float &PcZ = Pc.at<float>(2);
          
          mvCurrentKeys[i].mapPointCoords = {PcX, PcY, PcZ};
          


          

          if (pMP->Observations() > 0)
            mvbMap[i] = true;
          else
            mvbVO[i] = true;
        }
      }
    }
  }
  mState = static_cast<int>(pTracker->mLastProcessedState);
}


} // namespace ORB_SLAM2
