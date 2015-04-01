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

#ifndef ROS_IMG_SYNC_SYNC_IMPL_TRANSPORT_HANDLER_H
#define ROS_IMG_SYNC_SYNC_IMPL_TRANSPORT_HANDLER_H

#include "ros_msgs_sync/impl/sync_impl_handler.h"

// ROS headers
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/Image.h>

/**
* class SyncImageHandler
* It synchronises image topic callbacks (up to 8)
* Its callback is pure virtual so that it can be easily
* defined in a derived class
*/
class SyncImplTransportHandler :
  public SyncImplHandler<sensor_msgs::Image>
{

public:

  /**
  * Constructor.
  * Retrieve rosparam 'topics' as a list of image topics to synchronise
  *                   'transport' type of image transport. Default 'compressed'
  *                   'queue_size' size of synronisation queue
  */
  SyncImplTransportHandler();

  /**
  * Destructor.
  */
  ~SyncImplTransportHandler() {}

protected:

  typedef image_transport::ImageTransport It;
  typedef image_transport::SubscriberFilter SubsFil;

  virtual void initParam();
  virtual bool initSubs();
  virtual bool initSyncSubs();

  std::string _trp_hint;

  boost::shared_ptr<It> _img_trans;
  boost::ptr_vector<SubsFil> _img_subs;
};

#endif // ROS_IMG_SYNC_SYNC_IMPL_TRANSPORT_HANDLER_H
