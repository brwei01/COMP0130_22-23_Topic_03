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

#include <algorithm>
#include <boost/filesystem.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sysexits.h>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "Object.h"

namespace fs = ::boost::filesystem;
using namespace ::std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

void LoadBoundingBox(const string& strPathToDetectResult, 
                    std::vector<std::pair<std::vector<double>, int>>& detect_result);

int main(int argc, char **argv) {
  if (argc != 5) {
    cerr << endl << "Usage: " << argv[0] << " settings_files path_to_sequence results_file yolov5_detect_result_files " << endl;
    return EX_USAGE;
  }

  // Retrieve paths to images
  vector<string> vstrImageFilenames;
  vector<double> vTimestamps;
  string strFile = string(argv[2]) + "/rgb.txt";
  LoadImages(strFile, vstrImageFilenames, vTimestamps);

  int nImages = vstrImageFilenames.size();

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  string settingsFile =
      string(DEFAULT_MONO_SETTINGS_DIR) + string("/") + string(argv[1]);
  ORB_SLAM2::System SLAM(DEFAULT_ORB_VOCABULARY, settingsFile,
                         ORB_SLAM2::System::MONOCULAR, true);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;

  // Main loop
  int main_error = 0;
  std::thread runthread([&]() { // Start in new thread

    cv::Mat im;

    // ***************************************
    // MODIFICATIONS: ADD VARIABLE detect_result
    std::vector<std::pair<vector<double>, int>> detect_result;
    // END ADDING VARAIBLE
    // ***************************************

    for (int ni = 0; ni < nImages; ni++) {

      // ******************************* 
      // MODIFICATION: LOAD BOUNDING BOX
      string strPathToDetectionResult = argv[4] + std::to_string(vTimestamps[ni]) + ".txt"; // read detect result from yolov5
      LoadBoundingBox(strPathToDetectionResult, detect_result);
      if (detect_result.empty())
      {
        cerr << endl << "Failed to load bounding box" << endl;
        return 1;
      }
      // END MODIFICATION
      // ********************************

      // Read image from file
      im = cv::imread(string(argv[2]) + "/" + vstrImageFilenames[ni],
                      cv::IMREAD_UNCHANGED);
      double tframe = vTimestamps[ni];

      if (im.empty()) {
        cerr << endl
             << "Failed to load image at: " << string(argv[2]) << "/"
             << vstrImageFilenames[ni] << endl;
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

  // Start the visualization thread; this blocks until the SLAM system
  // has finished.
  SLAM.StartViewer();

  runthread.join();
  
  if (main_error != 0)
    return main_error;

  // Stop all threads
  SLAM.Shutdown();
  cout << "System Shutdown" << endl;

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
  // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  SLAM.SaveTrajectoryTUM(string(argv[3]));

  cout << "All done" << endl;
  
  return EX_OK;
}


void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps) {
  // Check the file exists
  if (fs::exists(strFile) == false) {
    cerr << "FATAL: Could not find the timestamp file " << strFile << endl;
    exit(0);
  }

  ifstream f;
  f.open(strFile.c_str());

  // skip first three lines
  string s0;
  getline(f, s0);
  getline(f, s0);
  getline(f, s0);
  if (f.good() == false) {
    cerr << "FATAL: Error reading the header from " << strFile << endl;
    exit(0);
  }

  while (!f.eof()) {
    string s;
    getline(f, s);
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      double t;
      string sRGB;
      ss >> t;
      vTimestamps.push_back(t);
      ss >> sRGB;
      vstrImageFilenames.push_back(sRGB);
    }
  }
}



// *******************************************
// MODIFICATIONS
void LoadBoundingBox(const string& strPathToDetectionResult, 
                    std::vector<std::pair<vector<double>, int>>& detect_result)
{
  ifstream infile;
  infile.open(strPathToDetectionResult);
  
  if(!infile.is_open())
  {
    cout<<"yolo detection result files failed to open"<<endl;
    exit(233);
  }
  vector<double> result_parameter;
  string line;
  while(getline(infile, line))
  {
    int sum = 0, num_bit = 0;
    for (char c: line)
    {
      if (c >= '0' && c <= '9')
      {
        num_bit = c - '0';
        sum = sum * 10 + num_bit;
      }
      else if (c = ' ')
      {
        result_parameter.push_back(sum);
        sum = 0;
        num_bit = 0;
      }
    }

    string idx_begin = "class:";
    int idx = line.find(idx_begin);
    string idx_end = "0.";
    int idx2 = line.find(idx_end);
    string class_label;
    for (int j = idx + 6; j < idx2-1; ++j)
    {
      class_label += line[j];
    }
    // cout << "**" << class_label << "**";

    int class_id = -1;//存入识别物体的种类
    if (class_label == "person") { //高动态物体:人,动物等
        class_id = 3;
    }

    if (class_label == "tv" ||   //低动态物体(在程序中可以假设为一直静态的物体):tv,refrigerator
        class_label == "refrigerator" || 
        class_label == "teddy bear") {
        class_id = 1;
    }

    if (class_label == "chair" || //中动态物体,在程序中不做先验动态静态判断
        class_label == "car"){
        class_id =2;
    }

    detect_result.emplace_back(result_parameter,class_id);
    result_parameter.clear();
    line.clear();
  }
  infile.close();
}
// END MODIFICATIONS
// *******************************************