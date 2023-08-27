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

#include <sys/socket.h>
#include <sys/un.h>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "Frame.h"

namespace fs = ::boost::filesystem;
using namespace std;


void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);
// ************************************************************
// MODIFICATION: ADD NEW FUNCTION
void LoadBoundingBox(const string& strPathToDetectResult, 
                    std::vector<std::pair<std::vector<double>, int>>& detect_result);
// END MODIFICATION
// ************************************************************


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
    // Main loop
    cv::Mat im;

    for (int ni = 0; ni < nImages; ni++) {

      // LOG: Image Series Number
      size_t lastSlashPos = vstrImageFilenames[ni].rfind('/');
      std::string ImageFilename = vstrImageFilenames[ni].substr(lastSlashPos + 1);
      size_t extensionPos = ImageFilename.rfind('.');
      std::string SeriesNumber = ImageFilename.substr(0, extensionPos);
      std::cout << "Processing Image NO.: " << SeriesNumber <<std::endl;
      // END LOGGING

      
      // ******************************* 
      // MODIFICATION: LOAD BOUNDING BOX
      string strPathToDetectionResult = argv[4] + std::to_string(vTimestamps[ni]) + ".txt"; // read detect result from yolov5
      // ***************************************
      // MODIFICATIONS: ADD VARIABLE detect_result
      // Clear the detect_result vector before loading new bounding boxes
      std::vector<std::pair<std::vector<double>, int>> detect_result;
      // END ADDING VARAIBLE
      // ***************************************  
      LoadBoundingBox(strPathToDetectionResult, detect_result);

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
void LoadBoundingBox(const string& strPathToDetectionResult, 
                    std::vector<std::pair<vector<double>, int>>& detect_result)
{
  ifstream infile;
  infile.open(strPathToDetectionResult);
  
  
  if(!infile.is_open())
  {
    std::cout<<R"(yolo detection result files failed to open at: )"<<strPathToDetectionResult<<std::endl;
    // exit(233);
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
    if (class_label == "person") {
        class_id = 3;
    }

    if (class_label == "tv" ||  
        class_label == "refrigerator" || 
        class_label == "teddy bear") {
        class_id = 1;
    }

    if (class_label == "bicycle" || 
        class_label == "car"){
        class_id = 2;
    }

    detect_result.emplace_back(result_parameter,class_id);
    result_parameter.clear();
    line.clear();
  }
  infile.close();
}
// END MODIFICATIONS
// *******************************************


/*
// FOR TEST
// =========================
void PrintDetectionResults(const std::vector<std::pair<std::vector<double>, int>>& detect_result)
{
  for (const auto& detection : detect_result)
  {
    const std::vector<double>& result_parameter = detection.first;
    int class_id = detection.second;

    // Print the detection parameters
    std::cout << "Detection Parameters: ";
    for (const double param : result_parameter)
    {
      std::cout << param << " ";
    }
    std::cout << std::endl;

    // Print the class ID
    std::cout << "Class ID: " << class_id << std::endl;
  }
}
// =========================
*/

