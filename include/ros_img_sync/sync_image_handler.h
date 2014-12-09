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

#ifndef ROS_IMG_EXTRACTOR_IMAGE_EXTRACTOR_H
#define ROS_IMG_EXTRACTOR_IMAGE_EXTRACTOR_H

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

// Boost headers
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/variant.hpp>

// OpenCV header
#include <opencv2/core/core.hpp>

namespace
{
  namespace sp = message_filters::sync_policies;
}


/**
* class SyncImageHandler
* It synchronises image topic callbacks (up to 8)
* Its callback is pure virtual so that it can be easily
* defined in a derived class
*/
class SyncImageHandler
{

public:

  /**
  * Constructor.
  * Retrieve rosparam 'topics' as a list of image topics to synchronise
  *                   'transport' type of image transport. Default 'compressed'
  *                   'queue_size' size of synronisation queue
  */
  SyncImageHandler();

  /**
  * Destructor.
  */
  ~SyncImageHandler() {}

protected:

  template<class T> struct Sync
  {
    typedef message_filters::Synchronizer<T> type;
    typedef boost::shared_ptr<message_filters::Synchronizer<T> > Ptr;
  };

  typedef sensor_msgs::Image I;
  typedef sensor_msgs::ImageConstPtr ICPtr;

  typedef image_transport::ImageTransport It;

  typedef image_transport::SubscriberFilter SubsFil;

  typedef boost::function<void (const std::vector<ICPtr>&)> callbackPtr;

  typedef sp::ApproximateTime<I, I> ApproxSync2;
  typedef sp::ApproximateTime<I, I, I> ApproxSync3;
  typedef sp::ApproximateTime<I, I, I, I> ApproxSync4;
  typedef sp::ApproximateTime<I, I, I, I, I> ApproxSync5;
  typedef sp::ApproximateTime<I, I, I, I, I, I> ApproxSync6;
  typedef sp::ApproximateTime<I, I, I, I, I, I, I> ApproxSync7;
  typedef sp::ApproximateTime<I, I, I, I, I, I, I, I> ApproxSync8;
  typedef sp::ApproximateTime<I, I, I, I, I, I, I, I, I> ApproxSync9;

  typedef boost::variant< Sync<ApproxSync2>::Ptr,
                          Sync<ApproxSync3>::Ptr,
                          Sync<ApproxSync4>::Ptr,
                          Sync<ApproxSync5>::Ptr,
                          Sync<ApproxSync6>::Ptr,
                          Sync<ApproxSync7>::Ptr,
                          Sync<ApproxSync8>::Ptr,
                          Sync<ApproxSync9>::Ptr > VariantApproxSync;

  /**
  * A pure virtual member.
  * @param vecImgPtr : std::vector< sensor_msgs::ImageConstPtr >
  *        callback has to be defined in derived class !
  */
  virtual void callback(const std::vector<ICPtr>& vecImgPtr) = 0;

  void wrapCallback(const ICPtr&, const ICPtr&,
                    const ICPtr&, const ICPtr&,
                    const ICPtr&, const ICPtr&,
                    const ICPtr&, const ICPtr&);

  void initSyncSubs();

  std::vector<std::string> _topics;
  std::string _trp_hint;

  boost::shared_ptr<It> _img_trans;
  boost::ptr_vector<SubsFil> _img_subs;
  boost::shared_ptr<VariantApproxSync> _approx_synchronizer;

  callbackPtr _callbackptr;

  ros::NodeHandle _nh;
  int _qsize;
};

#endif // ROS_IMG_EXTRACTOR_IMAGE_EXTRACTOR_H
