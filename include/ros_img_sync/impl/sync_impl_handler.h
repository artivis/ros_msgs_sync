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

#ifndef ROS_IMG_SYNC_SYNC_IMPL_HANDLER_H
#define ROS_IMG_SYNC_SYNC_IMPL_HANDLER_H

// ROS headers
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Boost headers
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/variant.hpp>
// Boost headers
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/assign.hpp>
#include <boost/filesystem.hpp>

// std header
#include <vector>

namespace
{
  namespace sp = message_filters::sync_policies;

  template <typename T>
  struct filtered_vector :
    public std::vector<boost::shared_ptr<T const> >
  {
      filtered_vector(const boost::shared_ptr<T const>& t)
      {
        if (t.get() != NULL)
          (*this)(t);
      }
      filtered_vector& operator()(const boost::shared_ptr<T const>& t)
      {
          if (t.get() != NULL)
            this->push_back(t);

          return *this;
      }
  };
}

/**
* class SyncImplHandler
* It synchronises ros messages topic callbacks (up to 8)
* Its callback is pure virtual so that it can be easily
* defined in a derived class
*/
template <typename M>
class SyncImplHandler
{

public:

  /**
  * Constructor.
  * Retrieve rosparam 'topics' as a list of image topics to synchronise
  *                   'queue_size' size of synronisation queue
  */
  SyncImplHandler();

  /**
  * Destructor.
  */
  ~SyncImplHandler() {}

  bool start();

  std::vector<std::string> getTopics()
    { return _topics; }

protected:

  template<class T> struct Sync
  {
    typedef message_filters::Synchronizer<T> type;
    typedef boost::shared_ptr<message_filters::Synchronizer<T> > Ptr;
  };

  typedef boost::shared_ptr<M const> MPtr;
  typedef message_filters::Subscriber<M> Sub;

  typedef boost::function<void (const std::vector<MPtr>&)> callbackPtr;

  typedef sp::ApproximateTime<M, M> ApproxSync2;
  typedef sp::ApproximateTime<M, M, M> ApproxSync3;
  typedef sp::ApproximateTime<M, M, M, M> ApproxSync4;
  typedef sp::ApproximateTime<M, M, M, M, M> ApproxSync5;
  typedef sp::ApproximateTime<M, M, M, M, M, M> ApproxSync6;
  typedef sp::ApproximateTime<M, M, M, M, M, M, M> ApproxSync7;
  typedef sp::ApproximateTime<M, M, M, M, M, M, M, M> ApproxSync8;
  typedef sp::ApproximateTime<M, M, M, M, M, M, M, M, M> ApproxSync9;

  typedef boost::variant< typename Sync<ApproxSync2>::Ptr,
                          typename Sync<ApproxSync3>::Ptr,
                          typename Sync<ApproxSync4>::Ptr,
                          typename Sync<ApproxSync5>::Ptr,
                          typename Sync<ApproxSync6>::Ptr,
                          typename Sync<ApproxSync7>::Ptr,
                          typename Sync<ApproxSync8>::Ptr,
                          typename Sync<ApproxSync9>::Ptr > VariantApproxSync;

  /**
  * A pure virtual member.
  * @param vecMPtr : std::vector< M >
  *        callback has to be defined in derived class !
  */
  virtual void callback(const std::vector<MPtr>& vecMPtr) = 0;

  void wrapCallback(const MPtr&, const MPtr&,
                    const MPtr&, const MPtr&,
                    const MPtr&, const MPtr&,
                    const MPtr&, const MPtr&);

  virtual void initParam();
  virtual bool initSubs();
  virtual bool initSyncSubs();

  std::vector<std::string> _topics;

  boost::ptr_vector<Sub> _msg_subs;
  boost::shared_ptr<VariantApproxSync> _approx_synchronizer;

  callbackPtr _callbackptr;

  ros::NodeHandle _nh;
  int _qsize;
};

template <class M>
SyncImplHandler<M>::SyncImplHandler() :
  _nh("~"),
  _qsize(10)
{

}

template <class M>
bool SyncImplHandler<M>::start()
{
  initParam();

  bool ok = initSubs();

  ok += initSyncSubs();

  _callbackptr = boost::bind(&SyncImplHandler<M>::callback, this, _1);

  return ok;
}

template <class M>
void SyncImplHandler<M>::initParam()
{
  std::vector<std::string> topics;
  _nh.getParam("topics", topics);
  _nh.param("queue_size", _qsize, _qsize);

  for (size_t i=0; i<topics.size(); ++i)
  {
    if (topics[i] != "none" && topics[i] != "")
    {
      ROS_INFO_STREAM("Listening topic : " << topics[i]);
      _topics.push_back(topics[i]);
    }
  }

  ROS_INFO_STREAM("Synchronizer queue size : " << _qsize);
}

template <class M>
bool SyncImplHandler<M>::initSubs()
{
  for (size_t i=0; i<_topics.size(); ++i)
    _msg_subs.push_back(new Sub(_nh, _topics[i], _qsize));

  if (_msg_subs.size()<2 || _msg_subs.size()>8)
  {
    ROS_ERROR("Can't listen to less than 2 topics neither more than 8 !");
    return false;
  }
  return true;
}

template <class M>
void SyncImplHandler<M>::wrapCallback(const MPtr& a, const MPtr& b,
                                      const MPtr& c, const MPtr& d,
                                      const MPtr& e, const MPtr& f,
                                      const MPtr& g, const MPtr& h)
{
  std::vector<MPtr> vecMPtr = filtered_vector<M>(a)(b)(c)(d)(e)(f)(g)(h);

  _callbackptr(vecMPtr);
}

template <class M>
bool SyncImplHandler<M>::initSyncSubs()
{
  switch (_msg_subs.size())
  {
    case 2:
      _approx_synchronizer.reset(new VariantApproxSync(typename Sync<ApproxSync2>::Ptr(new typename Sync<ApproxSync2>::type(ApproxSync2(_qsize),
                                                                                                                 _msg_subs[0], _msg_subs[1]))));
      boost::get<typename Sync<ApproxSync2>::Ptr>(*_approx_synchronizer)->registerCallback(
                    boost::bind(&SyncImplHandler<M>::wrapCallback, this, _1, _2, MPtr(), MPtr(), MPtr(), MPtr(), MPtr(), MPtr()));
      break;

    case 3:
      _approx_synchronizer.reset(new VariantApproxSync(typename Sync<ApproxSync3>::Ptr(new typename Sync<ApproxSync3>::type(ApproxSync3(_qsize),
                                                                                                   _msg_subs[0], _msg_subs[1], _msg_subs[2]))));
      boost::get<typename Sync<ApproxSync3>::Ptr>(*_approx_synchronizer)->registerCallback(
                    boost::bind(&SyncImplHandler<M>::wrapCallback, this, _1, _2, _3, MPtr(), MPtr(), MPtr(), MPtr(), MPtr()));
      break;

    case 4:
      _approx_synchronizer.reset(new VariantApproxSync(typename Sync<ApproxSync4>::Ptr(new typename Sync<ApproxSync4>::type(ApproxSync4(_qsize),
                                                                                     _msg_subs[0], _msg_subs[1], _msg_subs[2], _msg_subs[3]))));
      boost::get<typename Sync<ApproxSync4>::Ptr>(*_approx_synchronizer)->registerCallback(
                    boost::bind(&SyncImplHandler<M>::wrapCallback, this, _1, _2, _3, _4, MPtr(), MPtr(), MPtr(), MPtr()));
    break;

    case 5:
      _approx_synchronizer.reset(new VariantApproxSync(typename Sync<ApproxSync5>::Ptr(new typename Sync<ApproxSync5>::type(ApproxSync5(_qsize),
                                                                       _msg_subs[0], _msg_subs[1], _msg_subs[2], _msg_subs[3], _msg_subs[4]))));
      boost::get<typename Sync<ApproxSync5>::Ptr>(*_approx_synchronizer)->registerCallback(
                    boost::bind(&SyncImplHandler<M>::wrapCallback, this, _1, _2, _3, _4, _5, MPtr(), MPtr(), MPtr()));
      break;

    case 6:
      _approx_synchronizer.reset(new VariantApproxSync(typename Sync<ApproxSync6>::Ptr(new typename Sync<ApproxSync6>::type(ApproxSync6(_qsize),
                                                         _msg_subs[0], _msg_subs[1], _msg_subs[2], _msg_subs[3], _msg_subs[4], _msg_subs[5]))));
      boost::get<typename Sync<ApproxSync6>::Ptr>(*_approx_synchronizer)->registerCallback(
                    boost::bind(&SyncImplHandler<M>::wrapCallback, this, _1, _2, _3, _4, _5, _6, MPtr(), MPtr()));
      break;

    case 7:
      _approx_synchronizer.reset(new VariantApproxSync(typename Sync<ApproxSync7>::Ptr(new typename Sync<ApproxSync7>::type(ApproxSync7(_qsize),
                                           _msg_subs[0], _msg_subs[1], _msg_subs[2], _msg_subs[3], _msg_subs[4], _msg_subs[5], _msg_subs[6]))));
      boost::get<typename Sync<ApproxSync7>::Ptr>(*_approx_synchronizer)->registerCallback(
                    boost::bind(&SyncImplHandler<M>::wrapCallback, this, _1, _2, _3, _4, _5, _6, _7, MPtr()));
      break;

    case 8:
      _approx_synchronizer.reset(new VariantApproxSync(typename Sync<ApproxSync8>::Ptr(new typename Sync<ApproxSync8>::type(ApproxSync8(_qsize),
                             _msg_subs[0], _msg_subs[1], _msg_subs[2], _msg_subs[3], _msg_subs[4], _msg_subs[5], _msg_subs[6], _msg_subs[7]))));
      boost::get<typename Sync<ApproxSync8>::Ptr>(*_approx_synchronizer)->registerCallback(
                    boost::bind(&SyncImplHandler<M>::wrapCallback, this, _1, _2, _3, _4, _5, _6, _7, _8));
      break;
    }
  return true;
}

#endif // ROS_IMG_SYNC_SYNC_IMPL_HANDLER_H
