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
#include <regex>

#include <sys/socket.h>
#include <sys/un.h>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "Frame.h"

namespace fs = ::boost::filesystem;
using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

// MODIFICATION: ADD NEW FUNCTION
void LoadBoundingBox(const std::string& strPathToDetectionResult,
    vector<std::tuple<std::vector<double>, std::string, std::vector<std::vector<int>>>>& detect_result);

void ParseMaskCoords(const std::string& maskCoordsStr, std::vector<std::vector<int>>& maskCoords);

void PrintDetectionResults(const std::vector<std::tuple<std::vector<double>, std::string, std::vector<std::vector<int>>>>& detect_result);
// END MODIFICATION


int main(int argc, char **argv) {

  /*
  // Save the logs to file
  // create an ofstream obj to open the log file
  std::ofstream outputFile("/home/brwei01/Dev/SLAM_MASK/COMP0130_22-23_Topic_03/Coursework_03/Results/console_log.txt");
  // Redirect std::cout to the file stream
  std::streambuf* originalCoutBuffer = std::cout.rdbuf();
  std::cout.rdbuf(outputFile.rdbuf());
  */

  // OFFLINE -- requires extra statement 'yolov5_detect_result'
    if (argc != 5) {
    cerr << endl
          << "Usage: " << argv[0] << " settings_files path_to_sequence results_file yolact_detect_results" << endl;
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

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
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
      // frameInfo << SeriesNumber << std::endl;


      // MODIFICATION: LOAD BOUNDING BOX
      string fname;
      stringstream ss;
      ss << setfill('0') << setw(10) << ni;
      fname = ss.str() + ".txt";
      string strPathToDetectionResult = argv[4] + fname; // read detect result from yolov5

      // MODIFICATIONS: ADD VARIABLE detect_result
      // Clear the detect_result vector before loading new bounding boxes
      vector<tuple<vector<double>, string, vector<vector<int>>>> detect_result;

      // END ADDING VARAIBLE
      LoadBoundingBox(strPathToDetectionResult, detect_result);

      /*
      // this part annotated to avoid programme quitting where no detections seen
      if (detect_result.empty())
      {
        cerr << endl << "Failed to load bounding box" << endl;
        return 1;
      }
      */
      // END MODIFICATION LOAD BOUNDING BOX


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

  /*
  // Restore the original std::cout buffer
  std::cout.rdbuf(originalCoutBuffer);
  // Close the output file stream
  outputFile.close();
  */

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
    ss << setfill('0') << setw(10) << i;
    vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
  }
}


// TO USE THE FOLLOWING VERSION,
// YOLACT DETECT FILES MUST BE PROVIDED.
// MODIFICATIONS

void ParseMaskCoords(const std::string& maskCoordsStr, std::vector<std::vector<int>>& maskCoords)
{
    maskCoords.clear();

    // Define a regular expression pattern to match mask_coords
    std::regex pattern("\\[(\\d+), (\\d+)\\]");

    std::smatch matches;
    std::string::const_iterator searchStart(maskCoordsStr.cbegin());

    while (std::regex_search(searchStart, maskCoordsStr.cend(), matches, pattern))
    {
        std::vector<int> coordList;
        int x, y;

        x = std::stoi(matches[1]);
        y = std::stoi(matches[2]);

        coordList.push_back(x);
        coordList.push_back(y);

        maskCoords.push_back(coordList);

        // Update the searchStart position to continue searching
        searchStart = matches.suffix().first;
    }
}



void LoadBoundingBox(const std::string& strPathToDetectionResult,
    vector<std::tuple<std::vector<double>, std::string, std::vector<std::vector<int>>>>& detect_result)
{
    std::ifstream infile;
    infile.open(strPathToDetectionResult);

    if (!infile.is_open())
    {
        std::cout << R"(yolact detection result files failed to open at: )" << strPathToDetectionResult << std::endl;
        // exit(233);
    }

    vector<tuple<vector<double>, string, vector<vector<int>>>> result_parameters;

    std::string line;

    while (std::getline(infile, line))
    {
        // Initialize variables to store bounding_box and class_label
        vector<double> bounding_box;
        string class_label;

        // Parse the mask_coords part
        size_t maskCoordsPos = line.find("mask_coords:");
        if (maskCoordsPos != std::string::npos)
        {
            maskCoordsPos += 12; // Move past "mask_coords:"
            std::string maskCoordsStr = line.substr(maskCoordsPos);

            // Parse the bounding_box and class_label
            std::regex bboxPattern("left:(\\d+) top:(\\d+) right:(\\d+) bottom:(\\d+) class: (\\w+) score: (\\d+\\.\\d+)");
            std::smatch bboxMatches;

            if (std::regex_search(line, bboxMatches, bboxPattern))
            {
                // Extract bounding box values (left, top, right, bottom) and class label
                double left = std::stod(bboxMatches[1]);
                double top = std::stod(bboxMatches[2]);
                double right = std::stod(bboxMatches[3]);
                double bottom = std::stod(bboxMatches[4]);
                class_label = bboxMatches[5];

                // Populate the bounding_box vector
                bounding_box.push_back(left);
                bounding_box.push_back(top);
                bounding_box.push_back(right);
                bounding_box.push_back(bottom);
            }

            // Create a vector to store the parsed mask coordinates
            std::vector<std::vector<int>> maskCoords;
            ParseMaskCoords(maskCoordsStr, maskCoords);

            // Create a tuple with all the parsed information and add it to result_parameters
            result_parameters.push_back(std::make_tuple(bounding_box, class_label, maskCoords));
        }
    }

    // Move the result_parameters to detect_result
    detect_result = std::move(result_parameters);
    // PrintDetectionResults(detect_result);

    infile.close();
}



// FOR TEST
// =========================
void PrintDetectionResults(const vector<tuple<vector<double>, string, vector<vector<int>>>>& detect_result)
{
    for (const auto& detection : detect_result)
    {
        const vector<double>& result_parameters = std::get<0>(detection);
        const string& class_label = std::get<1>(detection);
        const vector<vector<int>>& mask_coords = std::get<2>(detection);

        // Print the detection parameters
        std::cout << "Detection Parameters: ";
        for (const double param : result_parameters)
        {
            std::cout << param << " ";
        }
        std::cout << std::endl;

        // Print the class label
        std::cout << "Class Label: " << class_label << std::endl;

        // Print the mask coordinates
        std::cout << "Mask Coordinates:" << std::endl;
        for (const auto& coords : mask_coords)
        {
            for (const int coord : coords)
            {
                std::cout << coord << " ";
            }
            std::cout << std::endl;
        }

        std::cout << std::endl; // Separate each detection with an empty line
    }
}


// =========================




