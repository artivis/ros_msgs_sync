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
* and save them in a given folder.
*/

#include "ros_img_sync/sync_image_handler.h"

// ROS headers
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

// BOOST headers
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/replace.hpp>

// OpenCV header
#include <opencv2/highgui/highgui.hpp>

/**
* class SyncImageExtractor
* It synchronises image topic callbacks (up to 8)
* Its callback save images named after their topic
*/
class SyncImageExtractor : public SyncImageHandler
{
public:

  /**
  * Constructor.
  * @see SyncImageHandler
  * Retrieve rosparam 'file_path' a complete folder path to save images
  */
  SyncImageExtractor() :
    SyncImageHandler(), // don't forget to call base constructor first
    _imgnum(0)
  {
    _nh.getParam("file_path", _filePath);

    // Reqieres a full path
    if (!boost::filesystem::is_directory(_filePath))
    {
      ROS_ERROR_STREAM("ERROR! Directory not found: " << _filePath);
      ROS_ERROR("Absolute path needed !");
      ros::shutdown();
    }

    for (int i=0; i<_topics.size(); ++i)
    {
      ROS_INFO_STREAM("Saving images from topic : " << _topics[i]
      << " in folder : " << _filePath);
      boost::replace_all(_topics[i], "/", "_");
    }
  }

  ~SyncImageExtractor() {}

protected:

  /**
  * A defined virtual member.
  * @param vecImgPtr : std::vector< sensor_msgs::ImageConstPtr >
  */
  virtual void callback(const std::vector<ICPtr>& vecImgPtr)
  {
    for (int i=0; i<vecImgPtr.size(); ++i)
    {
      if (vecImgPtr[i].use_count() > 0)
      {
        try
        {
          _image = cv_bridge::toCvShare(vecImgPtr[i], vecImgPtr[i]->encoding)->image;
        }
        catch (cv_bridge::Exception)
        {
          ROS_ERROR("Couldn't convert %s image", vecImgPtr[i]->encoding.c_str());
          return;
        }

        std::string filename = boost::str(boost::format( "%04d" ) % _imgnum )
          + "_" + _topics[i] + ".jpg";
        boost::filesystem::path fullPath(_filePath);
        fullPath /= filename;

        try
        {
          cv::imwrite(fullPath.c_str(), _image);
        }
        catch (cv::Exception)
        {
          ROS_ERROR_STREAM("Couldn't save image %s " << fullPath.c_str());
          return;
        }

        ROS_INFO_STREAM("Saved image : "<< filename <<" at "<< _filePath <<". \n");
      }
    }
    ++_imgnum;
  }

  std::string _filePath;
  int _imgnum;
  cv::Mat _image;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sync_img_extractor");

    SyncImageExtractor sync_cam;

    ros::spin();

    return 0;
}
