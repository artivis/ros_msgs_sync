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

/**
* class SyncImageDisplay
* It synchronises image topic callbacks (up to 8)
* Its callback store images in a member
*/
class SyncImageDisplay : public SyncImageHandler
{
public:

  /**
  * Constructor.
  * @see SyncImageHandler
  */
  SyncImageDisplay() :
    SyncImageHandler(), // don't forget to call base constructor first
    _new_img(false)
  {
    for (int i=0; i<_topics.size(); ++i)
      _images.push_back(cv::Mat());

  }

  ~SyncImageDisplay() {}

  /**
  * getTopics()
  * Returns topics that are listened
  */
  std::vector<std::string> getTopics()
  {
    return _topics;
  }

  /**
  * getImages()
  * Gets updated images from SyncImageDisplay object
  * @param images : std::vector< cv::Mat >
  *                 this vector is first cleared then
  *                 filled with images
  * @param timeout : max duration to wait for images
  */
  bool waitForImages(std::vector<cv::Mat>& images,
                     ros::Duration timeout = ros::Duration(0))
  {
    ros::Duration sleep(0.005);
    ros::Time start = ros::Time::now();

    _new_img = false;

    while (!_new_img)
    {
      sleep.sleep();
      if (timeout != ros::Duration(0))
        if ( (ros::Time::now() - start) > timeout )
          return false;
    }

    images.clear();

    boost::mutex::scoped_lock lock(img_mut);

    for (int i=0; i<_images.size(); ++i)
      images.push_back(_images[i].clone());

    return true;
  }

protected:

  /**
  * A defined virtual member.
  * @param vecImgPtr : std::vector< sensor_msgs::ImageConstPtr >
  */
  virtual void callback(const std::vector<ICPtr>& vecImgPtr)
  {
    boost::mutex::scoped_lock lock(img_mut);

    for (int i=0; i<vecImgPtr.size(); ++i)
    {
      if (vecImgPtr[i].use_count() > 0)
      {
        try
        {
          // Here we clone OpenCV Mat to avoid messing up with pointers
          _images[i] = cv_bridge::toCvShare(vecImgPtr[i],
                               vecImgPtr[i]->encoding)->image.clone();
        }
        catch (cv_bridge::Exception)
        {
          ROS_ERROR("Couldn't convert %s image", vecImgPtr[i]->encoding.c_str());
          _images[i] = cv::Mat();
        }
      }
    }

    _new_img = true;
  }

  boost::mutex img_mut;
  std::vector<cv::Mat> _images;
  bool _new_img;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sync_img_extractor");

  SyncImageDisplay sync_cam;

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
    new_img = sync_cam.waitForImages(images, ros::Duration(0));

    // If new images are grabbed
    // then display
    if (new_img)
    {
      for (int i=0; i<topics.size(); ++i)
      {
        if (!images[i].empty())
          cv::imshow(topics[i], images[i]);
      }
    }

    key = cv::waitKey(150);
  }

  return 0;
}
