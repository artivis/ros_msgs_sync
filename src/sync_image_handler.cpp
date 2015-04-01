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

#include "ros_msgs_sync/sync_image_handler.h"

SyncImageHandler::SyncImageHandler() :
  SyncImplHandler(),
  _new_mess(false)
{

}

bool SyncImageHandler::waitForImages(std::vector<sensor_msgs::Image>& images,
                                     ros::Duration timeout)
{
  _new_mess = false;

  images.clear();

  ros::Duration sleep(0, 10e-9/30); // wait for 1/30 sec
  ros::Time start = ros::Time::now();

  while (!_new_mess)
  {
    if (timeout != ros::Duration(0))
      if ( (ros::Time::now() - start).toSec() > timeout.toSec() )
        return false;

    sleep.sleep();
  }

  boost::mutex::scoped_lock lock(_mut);

  // TODO check copy constructor sensor_msgs::Image
  for (size_t i=0; i<_images.size(); ++i)
    images.push_back(_images[i]);

  return true;
}

//
// A pure virtual member.
// @param vecPcldPtr : std::vector< sensor_msgs::Image::Ptr >
//        callback has to be defined in derived class !
//
void SyncImageHandler::callback(const std::vector<MPtr>& vecMPtr)
{
  boost::mutex::scoped_lock lock(_mut);

  _images.clear();

  for (size_t i=0; i<vecMPtr.size(); ++i)
  {
    try
    {
      _images.push_back(*vecMPtr[i]);
    }
    catch (std::exception& e)
    {
      ROS_ERROR("Couldn't retrieve image : %s", e.what());
    }
  }

  _new_mess = true;
}
