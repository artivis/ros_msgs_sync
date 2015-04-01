#include "ros_img_sync/impl/sync_impl_transport_handler.h"

// Boost headers
#include <boost/bind.hpp>

SyncImplTransportHandler::SyncImplTransportHandler() :
  SyncImplHandler()
{

}

void SyncImplTransportHandler::initParam()
{
  std::vector<std::string> topics;
  _nh.getParam("topics", topics);
  _nh.param("queue_size", _qsize, (int)10);
  _nh.param("transport", _trp_hint, std::string("compressed"));

  for (size_t i=0; i<topics.size(); ++i)
  {
    if (topics[i] != "none" && topics[i] != "")
    {
      ROS_INFO_STREAM("Listening topic : " << topics[i]);
      _topics.push_back(topics[i]);
    }
  }

  ROS_INFO_STREAM("Synchronizer queue size : " << _qsize);
  ROS_INFO_STREAM("Image transport hint : " << _trp_hint);
}

bool SyncImplTransportHandler::initSubs()
{
  _img_trans.reset( new It( _nh ) );
  image_transport::TransportHints transportHint(_trp_hint);

  for (size_t i=0; i<_topics.size(); ++i)
    _img_subs.push_back(new SubsFil(*_img_trans, _topics[i], 1, transportHint));

  if (_img_subs.size()<2 || _img_subs.size()>8)
  {
    ROS_ERROR("Can't listen to less than 2 topics neither more than 8 !");
    return false;
  }
  return true;
}

bool SyncImplTransportHandler::initSyncSubs()
{
  switch (_img_subs.size())
  {
    case 2:
      _approx_synchronizer.reset(new VariantApproxSync(Sync<ApproxSync2>::Ptr(new Sync<ApproxSync2>::type(ApproxSync2(_qsize),
                                                                                               _img_subs[0], _img_subs[1]))));
      boost::get<Sync<ApproxSync2>::Ptr>(*_approx_synchronizer)->registerCallback(
                    boost::bind(&SyncImplTransportHandler::wrapCallback, this, _1, _2, MPtr(), MPtr(), MPtr(), MPtr(), MPtr(), MPtr()));
     break;

    case 3:
      _approx_synchronizer.reset(new VariantApproxSync(Sync<ApproxSync3>::Ptr(new Sync<ApproxSync3>::type(ApproxSync3(_qsize),
                                                                                 _img_subs[0], _img_subs[1], _img_subs[2]))));
      boost::get<Sync<ApproxSync3>::Ptr>(*_approx_synchronizer)->registerCallback(
                    boost::bind(&SyncImplTransportHandler::wrapCallback, this, _1, _2, _3, MPtr(), MPtr(), MPtr(), MPtr(), MPtr()));
      break;

    case 4:
      _approx_synchronizer.reset(new VariantApproxSync(Sync<ApproxSync4>::Ptr(new Sync<ApproxSync4>::type(ApproxSync4(_qsize),
                                                                   _img_subs[0], _img_subs[1], _img_subs[2], _img_subs[3]))));
      boost::get<Sync<ApproxSync4>::Ptr>(*_approx_synchronizer)->registerCallback(
                    boost::bind(&SyncImplTransportHandler::wrapCallback, this, _1, _2, _3, _4, MPtr(), MPtr(), MPtr(), MPtr()));
    break;

    case 5:
      _approx_synchronizer.reset(new VariantApproxSync(Sync<ApproxSync5>::Ptr(new Sync<ApproxSync5>::type(ApproxSync5(_qsize),
                                                     _img_subs[0], _img_subs[1], _img_subs[2], _img_subs[3], _img_subs[4]))));
      boost::get<Sync<ApproxSync5>::Ptr>(*_approx_synchronizer)->registerCallback(
                    boost::bind(&SyncImplTransportHandler::wrapCallback, this, _1, _2, _3, _4, _5, MPtr(), MPtr(), MPtr()));
      break;

    case 6:
      _approx_synchronizer.reset(new VariantApproxSync(Sync<ApproxSync6>::Ptr(new Sync<ApproxSync6>::type(ApproxSync6(_qsize),
                                       _img_subs[0], _img_subs[1], _img_subs[2], _img_subs[3], _img_subs[4], _img_subs[5]))));
      boost::get<Sync<ApproxSync6>::Ptr>(*_approx_synchronizer)->registerCallback(
                    boost::bind(&SyncImplTransportHandler::wrapCallback, this, _1, _2, _3, _4, _5, _6, MPtr(), MPtr()));
      break;

    case 7:
      _approx_synchronizer.reset(new VariantApproxSync(Sync<ApproxSync7>::Ptr(new Sync<ApproxSync7>::type(ApproxSync7(_qsize),
                         _img_subs[0], _img_subs[1], _img_subs[2], _img_subs[3], _img_subs[4], _img_subs[5], _img_subs[6]))));
      boost::get<Sync<ApproxSync7>::Ptr>(*_approx_synchronizer)->registerCallback(
                    boost::bind(&SyncImplTransportHandler::wrapCallback, this, _1, _2, _3, _4, _5, _6, _7, MPtr()));
      break;

    case 8:
      _approx_synchronizer.reset(new VariantApproxSync(Sync<ApproxSync8>::Ptr(new Sync<ApproxSync8>::type(ApproxSync8(_qsize),
           _img_subs[0], _img_subs[1], _img_subs[2], _img_subs[3], _img_subs[4], _img_subs[5], _img_subs[6], _img_subs[7]))));
      boost::get<Sync<ApproxSync8>::Ptr>(*_approx_synchronizer)->registerCallback(
                    boost::bind(&SyncImplTransportHandler::wrapCallback, this, _1, _2, _3, _4, _5, _6, _7, _8));
      break;
    }
  return true;
}
