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
 * MERCHANTABIeLITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */


#include <algorithm>
#include <boost/filesystem.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sysexits.h>
#include <sstream>
#include <string>

#include <sys/socket.h>
#include <sys/un.h>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "Frame.h"

namespace fs = ::boost::filesystem;
using namespace std;


void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

void ReadBoundingBox(const string& strPathToDetectionResult, 
                    std::vector<int>& frame_id, 
                    vector<string>& type, 
                    vector<vector<double>>& bbox_2d, 
                    vector<vector<vector<double>>>& bbox_3d,
                    vector<vector<double>>& bbox_2d_BV);

void LoadBoundingBoxBV(int frame_number,
                     std::vector<int>& nframe_id,
                     std::vector<std::string>& stype,
                     std::vector<std::vector<double>>& vbbox_2d, 
                     std::vector<std::vector<std::vector<double>>>& vbbox_3d,
                     std::vector<std::vector<double>>& vbbox_2d_BV,
                     std::vector<std::tuple<int, std::vector<double>, vector<vector<double>>, vector<double>, std::string>>& detect_result);

std::vector<std::vector<double>> Create3DBbox(double h, double w, double l, double x, double y, double z, double yaw);



int main(int argc, char **argv) {

  // Save the logs to file
  // create an ofstream obj to open the log file
  std::ofstream outputFile("/home/brwei01/Dev/COMP0130_22-23_Topic_03/Coursework_03/Results/console_log.txt");
  // Redirect std::cout to the file stream
  std::streambuf* originalCoutBuffer = std::cout.rdbuf();
  std::cout.rdbuf(outputFile.rdbuf());

  // OFFLINE -- requires extra statement 'yolov5_detect_result'
    if (argc != 5) {
    cerr << endl
          << "Usage: " << argv[0] << " settings_files path_to_sequence results_file 3d_detection_results" << endl;
    return EX_USAGE;
  }

  // Retrieve paths to images
  // store path to mono images and corresponding timestamps 
  vector<string> vstrImageFilenames;
  vector<double> vTimestamps;
  // "string strAssciationFilename = string(argv[2])" from terminal input to get associated filename
  // to load these information
  LoadImages(string(argv[2]), vstrImageFilenames, vTimestamps);

  int nImages = vstrImageFilenames.size();

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  string settingsFile = string(DEFAULT_MONO_SETTINGS_DIR) + string(argv[1]);
  ORB_SLAM2::System SLAM(DEFAULT_ORB_VOCABULARY, settingsFile,
                         ORB_SLAM2::System::MONOCULAR, true);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;

  int main_error = 0;
  std::thread runthread([&]() { // Start in new thread

    // MODIFICATION: LOAD ANNOTATED GT DATA: 2DBBX, 3DBBX, CLASS
    
    string strPathToDetectionResult = argv[4];

    // READ ALL ANNOTATED DATA
    std::vector<int> nframe_id;
    std::vector<std::string> stype;
    std::vector<std::vector<double>> vbbox_2d; 
    std::vector<std::vector<std::vector<double>>> vbbox_3d;
    std::vector<std::vector<double>> vbbox_2d_BV;
    // read data into these variables defined above
    ReadBoundingBox(strPathToDetectionResult, nframe_id, stype, vbbox_2d, vbbox_3d, vbbox_2d_BV);

    // Main loop: load images
    cv::Mat im;
    for (int ni = 0; ni < nImages; ni++) {
      // LOG: Image Series Number
      size_t lastSlashPos = vstrImageFilenames[ni].rfind('/');
      std::string ImageFilename = vstrImageFilenames[ni].substr(lastSlashPos + 1);
      size_t extensionPos = ImageFilename.rfind('.');
      std::string SeriesNumber = ImageFilename.substr(0, extensionPos);
      std::cout << "Processing Image NO.: " << SeriesNumber <<std::endl;
      // END LOGGING
      
      // ADD VARIABLE detect_result
      // Clear the detect_result vector before loading new bounding boxes
      std::vector<std::tuple<int, std::vector<double>, std::vector<std::vector<double>>, std::vector<double> ,std::string>> detect_result;

      LoadBoundingBoxBV(ni, nframe_id, stype, vbbox_2d, vbbox_3d, vbbox_2d_BV, detect_result);


      /*
      // this part annotated to avoid programme quitting where no detections seen
      // but can be left here for further developments
      if (detect_result.empty())
      {
        cerr << endl << "Failed to load bounding box" << endl;
        return 1;
      }
      */
      // END MODIFICATION
      // ********************************


      // Read image from file
      im = cv::imread(vstrImageFilenames[ni], cv::IMREAD_UNCHANGED);
      double tframe = vTimestamps[ni];

      // make sure the image is valid
      if (im.empty()) {
        cerr << endl
             << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
        main_error = EX_DATAERR;
        break;
      }

      if (SLAM.isFinished() == true) {
	  break;
      }
      
      chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

      // Pass the image to the SLAM system
      SLAM.TrackMonocular(im, tframe, detect_result);

      chrono::steady_clock::time_point t2 = chrono::steady_clock::now();

      double ttrack =
          chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();

      vTimesTrack[ni] = ttrack;

      // Wait to load the next frame
      double T = 0;
      if (ni < nImages - 1)
        T = vTimestamps[ni + 1] - tframe;
      else if (ni > 0)
        T = tframe - vTimestamps[ni - 1];

      if (ttrack < T)
        this_thread::sleep_for(chrono::duration<double>(T - ttrack));
    }
    SLAM.StopViewer();
  });
  
  SLAM.StartViewer();

  cout << "Viewer started, waiting for thread." << endl;
  runthread.join();
  if (main_error != 0)
    return main_error;
  cout << "Tracking thread joined..." << endl;

  // Stop all threads
  SLAM.Shutdown();

  // Tracking time statistics
  sort(vTimesTrack.begin(), vTimesTrack.end());
  float totaltime = 0;
  for (int ni = 0; ni < nImages; ni++) {
    totaltime += vTimesTrack[ni];
  }
  cout << "-------" << endl << endl;
  cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
  cout << "mean tracking time: " << totaltime / nImages << endl;

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM("Results/KeyFrameTrajectory.txt");
  SLAM.SaveTrajectoryTUM(string(argv[3]));

  // Save to KITTI pose file
  //SLAM.SaveTrajectoryKITTI(string(argv[3]));

  // Restore the original std::cout buffer
  std::cout.rdbuf(originalCoutBuffer);
  // Close the output file stream
  outputFile.close();

  return EX_OK;
}

void LoadImages(const string &strPathToSequence,
                vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps) {

  // Check the file exists
  if (fs::exists(strPathToSequence) == false) {
    cerr << "FATAL: Could not find the timestamp file " << strPathToSequence
         << endl;
    exit(0);
  }

  ifstream fTimes;
  string strPathTimeFile = strPathToSequence + "/times.txt";
  // string strPathTimeFile = strPathToSequence + "/times_f.txt";
  if (fs::exists(strPathTimeFile) == false) {
    cerr << "FATAL: Could not find the timestamp file " << strPathTimeFile
         << endl;
    exit(0);
  }
  fTimes.open(strPathTimeFile.c_str());
  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      double t;
      ss >> t;
      vTimestamps.push_back(t);
    }
  }

  string strPrefixLeft = strPathToSequence + "/image_0/";

  const int nTimes = vTimestamps.size();
  vstrImageFilenames.resize(nTimes);

  for (int i = 0; i < nTimes; i++) {
    stringstream ss;
    ss << setfill('0') << setw(6) << i;
    vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
  }
}


// *******************************************
// TO USE THE FOLLOWING VERSION,
// YOLO DETECT FILES MUST BE PROVIDED.
// MODIFICATIONS
void LoadBoundingBoxBV(int frame_number,
                     std::vector<int>& nframe_id,
                     std::vector<std::string>& stype,
                     std::vector<std::vector<double>>& vbbox_2d, 
                     std::vector<std::vector<std::vector<double>>>& vbbox_3d,
                     std::vector<std::vector<double>>& vbbox_2d_BV,
                     std::vector<std::tuple<int, std::vector<double>, vector<vector<double>>, vector<double>, std::string>>& detect_result)

{
    // Clear the detect_result vector before loading new bounding boxes
    detect_result.clear();
    // Iterate through the nframe_id vector to find matches
    for (size_t i = 0; i < nframe_id.size(); ++i) {
        if (nframe_id[i] == frame_number) {
            // Match found, add corresponding data to detect_result
            detect_result.emplace_back(nframe_id[i], vbbox_2d[i], vbbox_3d[i], vbbox_2d_BV[i], stype[i]);
        }
    }
}

// function to read boundint box
void ReadBoundingBox(const string& strPathToDetectionResult, 
                     vector<int>& frame_id, 
                     vector<string>& type, 
                     vector<vector<double>>& bbox_2d, 
                     vector<vector<vector<double>>>& bbox_3d,
                     vector<vector<double>>& bbox_2d_BV)
{
  ifstream infile;
  infile.open(strPathToDetectionResult);
  
  if(!infile.is_open())
  {
    std::cout<<R"(yolo detection result files failed to open at: )" << strPathToDetectionResult << std::endl;
    exit(233);
  }
    std::string line; 
    while (std::getline(infile, line)) {
      std::istringstream iss(line);
      int frame_val, track_id, truncated, occluded;
      std::string type_val;
      double alpha, bbox_left_val, bbox_top_val, bbox_right_val, bbox_bottom_val;
      double height_val, width_val, length_val, pos_x_val, pos_y_val, pos_z_val, rot_y_val;


      if (!(iss >> frame_val >> track_id >> type_val >> truncated >> occluded >> alpha
          >> bbox_left_val >> bbox_top_val >> bbox_right_val >> bbox_bottom_val
          >> height_val >> width_val >> length_val >> pos_x_val >> pos_y_val >> pos_z_val >> rot_y_val)) {
          std::cerr << "Error reading line." << std::endl;
          continue;
      }

      // save all frame ids
      frame_id.push_back(frame_val);
      // save all types
      type.push_back(type_val);

      // save all 2d bboxes
      std::vector<double> bbox_2d_val;
      bbox_2d_val.push_back(bbox_left_val);
      bbox_2d_val.push_back(bbox_top_val);
      bbox_2d_val.push_back(bbox_right_val);
      bbox_2d_val.push_back(bbox_bottom_val);

      bbox_2d.push_back(bbox_2d_val);

      // save all calculated 3d bboxes 
      std::vector<std::vector<double>> bbox_3d_val;
      bbox_3d_val = Create3DBbox(height_val, width_val, length_val, pos_x_val, pos_y_val, pos_z_val, rot_y_val);
      bbox_3d.push_back(bbox_3d_val);

      std::vector<double> bbox_2d_BV_val;
      bbox_2d_BV_val = {0,0,0,0};
      bbox_2d_BV.push_back(bbox_2d_BV_val);
    }
}



// function to create 3d bounding box:
std::vector<std::vector<double>> Create3DBbox(double h, double w, double l, double x, double y, double z, double yaw) 
{
    std::vector<std::vector<double>> bbox_3d(8, std::vector<double>(3, 0.0));

    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);

    double x_corners[] = { l / 2,  l / 2, -l / 2, -l / 2,  l / 2,  l / 2, -l / 2, -l / 2 };
    double y_corners[] = {   0.0,    0.0,   0.0,    0.0, -h,     -h,    -h,     -h   };
    double z_corners[] = {  w / 2, -w / 2, -w / 2,  w / 2,  w / 2, -w / 2, -w / 2,  w / 2 };

    for (int i = 0; i < 8; ++i) {
        double rotated_x = cos_yaw * x_corners[i] - sin_yaw * z_corners[i] + x;
        double rotated_y = y_corners[i] + y;
        double rotated_z = sin_yaw * x_corners[i] + cos_yaw * z_corners[i] + z;

        bbox_3d[i][0] = rotated_x;
        bbox_3d[i][1] = rotated_y;
        bbox_3d[i][2] = rotated_z;
    }
    
    return bbox_3d;
}


