/*
* Software License Agreement (Modified BSD License)
*
* Copyright (c) 2014, PAL Robotics, S.L.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of PAL Robotics, S.L. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
/** \author Jeremie Deray. */

/**
* This function sample shows how to inherit from class SyncImageHandler
* and define pure virtual function SyncImageHandler::callback()
*
* It subscribes to a given list of image topics, synchronises them
* and display them in OpenCV windows.
*/

#include "ros_img_sync/sync_image_handler.h"

// ROS headers
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

// BOOST headers
#include <boost/thread/mutex.hpp>

// OpenCV header
#include <opencv2/highgui/highgui.hpp>

class SyncCVImageHandler :
  public SyncImageHandler
{

public:

  SyncCVImageHandler() :
    SyncImageHandler(),
    _encoding("bgr8")
  {
    bool isStart = start();

    _nh.param("encoding", _encoding, _encoding);

    if (!isStart)
    {
      ROS_ERROR("Something went wrong.");
      ROS_ERROR("Shutting down.");
      _nh.shutdown();
    }
  }

  ~SyncCVImageHandler() {}

  bool getImages(std::vector<cv::Mat>& cvimage, ros::Duration timer = ros::Duration(0))
  {
    std::vector<sensor_msgs::Image> images;

    if (!waitForImages(images, timer))
      return false;

    cvimage.clear();

    for (size_t i=0; i<images.size(); ++i)
    {
      try
      {
        cvimage.push_back(cv_bridge::toCvShare(boost::make_shared<sensor_msgs::Image>(images[i]),
                                               _encoding)->image.clone());
      }
      catch (cv_bridge::Exception)
      {
        ROS_ERROR("Couldn't convert image from %s to %s",
                  _encoding.c_str(), images[i].encoding.c_str());
        return false;
      }
    }
    return true;
  }

private:

  std::string _encoding;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sync_img_extractor");

  SyncCVImageHandler sync_cam;

  std::vector<cv::Mat> images;
  char key = -1;
  bool new_img = false;

  // Get topics that are listened
  std::vector<std::string> topics = sync_cam.getTopics();

  // Init OpenCV windows
  // named after topics
  for (int i=0; i<topics.size(); ++i)
    cv::namedWindow(topics[i], cv::WINDOW_NORMAL);

  // Use asynchronous spinner
  // so that listener keeps listening
  // while waiting for images
  ros::AsyncSpinner aspin(1);
  aspin.start();

  // 27 stands for key 'ESC'
  while (ros::ok() && key != 27)
  {
    // Get all images
    new_img = sync_cam.getImages(images, ros::Duration(0));

    // If new images are grabbed
    // then display
    if (new_img)
      for (size_t i=0; i<images.size(); ++i)
        if (!images[i].empty())
          cv::imshow(topics[i], images[i]);

    key = cv::waitKey(10);
  }

  return 0;
}
