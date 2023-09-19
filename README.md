Forked from ORB-SLAM2 modified by UCL https://github.com/UCL/COMP0130_22-23_Topic_03.git


# About

This repository contains the support material for Topic 03 of [COMP0130 Robot Vision and Navigation (22/23)](https://moodle.ucl.ac.uk/course/view.php?id=30087)

See the [wiki](https://github.com/UCL/COMP0130_22-23_Topic_03/wiki) for further details.


# yolov5 modi
the modifications made to yolov5 library
these include a socket server version 


# Running the UNIX socket version
run the following in your terminal, it is necessary to save the confidence level:\n
```
cd path_to_yolov5_folder
python detect.py --source path_to_data --save_txt --save_conf
```
when it prompts "waiting for connection", start another terminal window and run:\n
```
cd path_to_orbslam_folder
./Install/bin/mono_kitti KITTI$.yaml ResultFile_path
```


