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


// ******************************************
// TO MAKE FULLY ONLINE USING SOCKET SERVER
// replace the LoadBoundingBoxFromPython method
void MakeDetect_result(vector<std::pair<vector<double>,int>>& detect_result, int sockfd);

void LoadBoundingBoxFromPython(const string& resultFromPython, std::pair<vector<double>, int>& detect_result);
// END: TO MAKE FULLY ONLINE
// *******************************************
void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);


/*
// ************************************************************
// THIS IS THE OFFLINE VERSION
// MODIFICATION: ADD NEW FUNCTION
void LoadBoundingBox(const string& strPathToDetectResult, 
                    std::vector<std::pair<std::vector<double>, int>>& detect_result);
// END MODIFICATION
// ************************************************************
*/


int main(int argc, char **argv) {

  // SOCKET INITIALIZATION
  int sockfd;
  int len;
  struct sockaddr_un address;
  int result;
  int i, byte;
  char send_buf[128], ch_recv[1024];

  if((sockfd = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
  {
    perror("socket");
    exit(EXIT_FAILURE);
  }

  // setup server_address
  address.sun_family = AF_UNIX;
  strcpy(address.sun_path, "/home/borui/Dev/server_socket");
  len = sizeof(address);

  result = connect(sockfd, (struct sockaddr *)&address, len);

  if (result == -1)
  {
    printf("please ensure the server is up\n");
    perror("connect");
    exit(EXIT_FAILURE);
  }
  // END SOCKET SERVER INIT

  if(argc != 4) {
    cerr << endl
          << "Usage: " << argv[0] << " setting_files path_to_sequence results_file_dir" << endl;
    return EX_USAGE;
  }

  // ==================================
  // OFFLINE -- requires extra statement 'yolov5_detect_result'
  /*
    if (argc != 5) {
    cerr << endl
          << "Usage: " << argv[0] << " settings_files path_to_sequence results_file yolov5_detect_results" << endl;
    return EX_USAGE;
  }
  */
  // ===================================


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

      std::vector<std::pair<std::vector<double>, int>> detect_result;
      MakeDetect_result(detect_result, sockfd);
      
      // ========================================
      // USED FOR OFFLINE VERSION
      /*
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
      */

      /*
      // this part annotated to avoid programme quitting where no detections seen
      if (detect_result.empty())
      {
        cerr << endl << "Failed to load bounding box" << endl;
        return 1;
      }
      */

      // END MODIFICATION
      // ********************************
      // ====================================================


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
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  SLAM.SaveTrajectoryTUM(string(argv[3]));

  // Save to KITTI pose file
  //SLAM.SaveTrajectoryKITTI(string(argv[3]));


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




// ***************************************************************
// TO MAKE FULLY ONLINE USING SOCKET SERVER
// ADDING 2 FUNCTIONS: 
// LoadBoundingBoxFromPython, MakeDetect_result
void LoadBoundingBoxFromPython(const string& resultFromPython, std::pair<vector<double>, int>& detect_result)
{
  if(resultFromPython.empty())
  {
    cerr << "no string from python!" << endl;
  }

  cout << "running LoadBoundingBoxFromPython" << endl;

  vector<double> result_parameter;
  int sum = 0; 
  int num_bit = 0;

  int idx_bbxEnd = resultFromPython.find("class:");
  for(char c: resultFromPython.substr(0,idx_bbxEnd))
  {
    // read nums. e.g. 780 = ((7*10 + 8)*10) + 4;
    if(c >= '0' && c <= '9')
    {
      num_bit = c - '0';
      sum = sum * 10 + num_bit;
    }
    else if (c == ' ')
    {
      result_parameter.push_back(sum);
      sum = 0;
      num_bit = 0;
    }
  }

  detect_result.first = result_parameter;
  cout << "detect_result.first size is: " << detect_result.first.size() << endl;

  cout << "result parameter: ";
  for (const double& value : result_parameter)
  {
    cout << " " << value;
  }
  cout << endl;

  string idx_begin = "class:"; // read the class of the object;
  int idx = resultFromPython.find(idx_begin);
  string idx_end = "0.";

  int idx2 = resultFromPython.find(idx_end);
  string class_label;

  for (int j = idx + 6; j < idx2-1; ++j)
  {
    class_label += resultFromPython[j];
  }

  int class_id = -1; // store the class of obj detected

  if (class_label == "tv" ||
    class_label == "refrigerator" ||
    class_label == "teddy bear" ||
    class_label == "laptop"){
      class_id = 1;
    }

  if (class_label == "chair" ||
  class_label == "car"){
    class_id = 2;
  }

  if (class_label == "person"){
    class_id = 3;
  }

  detect_result.second = class_id;
  cout << "LoadBoundBoxFromPython class id is: " << class_id << endl;

}


void MakeDetect_result(vector<std::pair<vector<double>,int>>& detect_result, int sockfd)
{
  detect_result.clear();
  
  std::pair<vector<double>, int> detect_result_str;
  int byte;
  char send_buf[128], ch_recv[1024];

  sprintf(send_buf, "ok"); // sprintf sends the message to send_buf
  if((byte=write(sockfd, send_buf, sizeof(send_buf)))==-1)
  {
    perror("write");
    exit(EXIT_FAILURE);
  }

  if((byte=read(sockfd, &ch_recv, 1000))==-1)
  {
    perror("read");
    exit(EXIT_FAILURE);
  }
  // cout << "**ch_recv is : \n" << ch_recv << endl;

  char *ptr;
  ptr = strtok(ch_recv, "*"); // str split
  while(ptr != NULL)
  {
    printf("ptr=%s\n", ptr);

    if (strlen(ptr) > 20)
    {
      // cout << strlen(ptr) << endl;
      string ptr_str = ptr;
      LoadBoundingBoxFromPython(ptr_str, detect_result_str);
    }

    detect_result.emplace_back(detect_result_str);
    // cout <<   "hh: " << ptr_str << endl;
    ptr = strtok(NULL, "*");
    }

    // cout << "detect_result size is: " << detect_result.size() << endl;
    // for (int k = 0; k < detect_result.size(); ++k)
    // cout << "detect_result is: \n" << detect_result[k].second << endl;
}
// END: TO MAKE FULLY ONLINE
// **********************************************************************************



/*
// *******************************************
// TO USE THE FOLLOWING VERSION,
// YOLO DETECT FILES MUST BE IN PLACE.
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