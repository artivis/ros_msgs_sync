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

#include "ros_img_sync/sync_image_handler.h"

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

SyncImageHandler::SyncImageHandler() :
  _nh("~")
{
  std::vector<std::string> topics;
  _nh.getParam("topics", topics);
  _nh.param("transport", _trp_hint, std::string("compressed"));
  _nh.param("queue_size", _qsize, (int)10);

  _img_trans.reset( new It( _nh ) );

  image_transport::TransportHints transportHint(_trp_hint);

  for (int i=0; i<topics.size(); ++i)
  {
    if (topics[i] != "none" && topics[i] != "")
    {
      _img_subs.push_back(new SubsFil(*_img_trans, topics[i], 1, transportHint));
      _topics.push_back(topics[i]);
    }
  }

  if (_img_subs.size()<2 || _img_subs.size()>8)
  {
    ROS_ERROR("Can't listen to less than 2 topics neither more than 8 !");
    ros::shutdown();
  }

  ROS_INFO_STREAM("Image transport hint : " << _trp_hint);
  ROS_INFO_STREAM("Synchronizer queue size : " << _qsize);

  initSyncSubs();

  _callbackptr = boost::bind(&SyncImageHandler::callback, this, _1);
}

void SyncImageHandler::wrapCallback(const ICPtr& a, const ICPtr& b,
                                    const ICPtr& c, const ICPtr& d,
                                    const ICPtr& e, const ICPtr& f,
                                    const ICPtr& g, const ICPtr& h)
{
  std::vector<ICPtr> vecICPtr = boost::assign::list_of<ICPtr>(a)(b)(c)(d)(e)(f)(g)(h);

  _callbackptr(vecICPtr);
}

void SyncImageHandler::initSyncSubs()
{
  switch (_img_subs.size())
  {
    case 2:
      _approx_synchronizer.reset(new VariantApproxSync(Sync<ApproxSync2>::Ptr(new Sync<ApproxSync2>::type(ApproxSync2(_qsize),
                                                                                                        _img_subs[0], _img_subs[1]))));
      boost::get<Sync<ApproxSync2>::Ptr>(*_approx_synchronizer)->registerCallback(
                    (boost::bind(&SyncImageHandler::wrapCallback, this, _1, _2, ICPtr(), ICPtr(), ICPtr(), ICPtr(), ICPtr(), ICPtr())));
//  Make it happen
//       boost::bind(&SyncImageHandler::callback, this, boost::assign::list_of<ICPtr>(_1)(_2)(ICPtr())(ICPtr())(ICPtr())(ICPtr())(ICPtr())(ICPtr())) );
//       boost::bind(&SyncImageHandler::callback, this, boost::assign::list_of<ICPtr>(_1)(_2)(ICPtr())(ICPtr())(ICPtr())(ICPtr())(ICPtr())(ICPtr()).convert_to_container<std::vector<ICPtr> >() ) );
      break;

    case 3:
      _approx_synchronizer.reset(new VariantApproxSync(Sync<ApproxSync3>::Ptr(new Sync<ApproxSync3>::type(ApproxSync3(_qsize),
                                                                                            _img_subs[0], _img_subs[1], _img_subs[2]))));
      boost::get<Sync<ApproxSync3>::Ptr>(*_approx_synchronizer)->registerCallback(
                    boost::bind(&SyncImageHandler::wrapCallback, this, _1, _2, _3, ICPtr(), ICPtr(), ICPtr(), ICPtr(), ICPtr()));
      break;

    case 4:
      _approx_synchronizer.reset(new VariantApproxSync(Sync<ApproxSync4>::Ptr(new Sync<ApproxSync4>::type(ApproxSync4(_qsize),
                                                                                _img_subs[0], _img_subs[1], _img_subs[2], _img_subs[3]))));
      boost::get<Sync<ApproxSync4>::Ptr>(*_approx_synchronizer)->registerCallback(
                    boost::bind(&SyncImageHandler::wrapCallback, this, _1, _2, _3, _4, ICPtr(), ICPtr(), ICPtr(), ICPtr()));
    break;

    case 5:
      _approx_synchronizer.reset(new VariantApproxSync(Sync<ApproxSync5>::Ptr(new Sync<ApproxSync5>::type(ApproxSync5(_qsize),
                                                                    _img_subs[0], _img_subs[1], _img_subs[2], _img_subs[3], _img_subs[4]))));
      boost::get<Sync<ApproxSync5>::Ptr>(*_approx_synchronizer)->registerCallback(
                    boost::bind(&SyncImageHandler::wrapCallback, this, _1, _2, _3, _4, _5, ICPtr(), ICPtr(), ICPtr()));
      break;

    case 6:
      _approx_synchronizer.reset(new VariantApproxSync(Sync<ApproxSync6>::Ptr(new Sync<ApproxSync6>::type(ApproxSync6(_qsize),
                                                        _img_subs[0], _img_subs[1], _img_subs[2], _img_subs[3], _img_subs[4], _img_subs[5]))));
      boost::get<Sync<ApproxSync6>::Ptr>(*_approx_synchronizer)->registerCallback(
                    boost::bind(&SyncImageHandler::wrapCallback, this, _1, _2, _3, _4, _5, _6, ICPtr(), ICPtr()));
      break;

    case 7:
      _approx_synchronizer.reset(new VariantApproxSync(Sync<ApproxSync7>::Ptr(new Sync<ApproxSync7>::type(ApproxSync7(_qsize),
                                            _img_subs[0], _img_subs[1], _img_subs[2], _img_subs[3], _img_subs[4], _img_subs[5], _img_subs[6]))));
      boost::get<Sync<ApproxSync7>::Ptr>(*_approx_synchronizer)->registerCallback(
                    boost::bind(&SyncImageHandler::wrapCallback, this, _1, _2, _3, _4, _5, _6, _7, ICPtr()));
      break;

    case 8:
      _approx_synchronizer.reset(new VariantApproxSync(Sync<ApproxSync8>::Ptr(new Sync<ApproxSync8>::type(ApproxSync8(_qsize),
                                _img_subs[0], _img_subs[1], _img_subs[2], _img_subs[3], _img_subs[4], _img_subs[5], _img_subs[6], _img_subs[7]))));
      boost::get<Sync<ApproxSync8>::Ptr>(*_approx_synchronizer)->registerCallback(
                    boost::bind(&SyncImageHandler::wrapCallback, this, _1, _2, _3, _4, _5, _6, _7, _8));
      break;
    }
}

