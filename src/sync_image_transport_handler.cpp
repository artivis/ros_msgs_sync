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

#include "ros_img_sync/sync_image_transport_handler.h"

// ROS headers
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Boost headers
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/assign.hpp>
#include <boost/filesystem.hpp>

SyncImageTransportHandler::SyncImageTransportHandler() :
  SyncImplTransportHandler(),
  _new_img(false)
{  

}

void SyncImageTransportHandler::callback(const std::vector<MPtr>& vecMPtr)
{
  boost::mutex::scoped_lock lock(_mut);

  _images.clear();

  for (size_t i=0; i<vecMPtr.size(); ++i)
  {
    try
    {
      // Here we clone OpenCV Mat to avoid messing up with pointers
      _images.push_back(cv_bridge::toCvShare(vecMPtr[i],
                        vecMPtr[i]->encoding)->image.clone());
    }
    catch (cv_bridge::Exception)
    {
      ROS_ERROR("Couldn't convert %s image", vecMPtr[i]->encoding.c_str());
    }
  }

  _new_img = true;
}

/**
* waitForImages()
* Gets updated images from SyncImageDisplay object
* @param images : std::vector< cv::Mat >
* this vector is first cleared then
* filled with images
* @param timeout : max duration to wait for images
*/
bool SyncImageTransportHandler::waitForImages(std::vector<cv::Mat>& images,
                                              ros::Duration timeout)
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

  boost::mutex::scoped_lock lock(_mut);

  for (int i=0; i<_images.size(); ++i)
    images.push_back(_images[i].clone());

  return true;
}
