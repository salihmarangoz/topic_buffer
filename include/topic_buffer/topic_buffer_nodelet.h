#ifndef TOPIC_BUFFER_NODELET_H
#define TOPIC_BUFFER_NODELET_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <nodelet/nodelet.h>
#include <topic_tools/shape_shifter.h>
#include <mutex>
#include <queue>
#include <chrono>
#include <thread>

namespace topic_buffer
{

struct StampedMessage
{
  StampedMessage(const boost::shared_ptr<topic_tools::ShapeShifter const> msg_, std::chrono::system_clock::time_point publish_time_): msg(msg_), publish_time(publish_time_){}
  const boost::shared_ptr<topic_tools::ShapeShifter const> msg;
  std::chrono::system_clock::time_point publish_time;
};

class TopicBufferNodelet : public nodelet::Nodelet
{
public:
  TopicBufferNodelet();
  ~TopicBufferNodelet();
  virtual void onInit();
  bool clearBufferCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  ros::Publisher advertise(boost::shared_ptr<topic_tools::ShapeShifter const> msg,const std::string& topic);
  virtual void messageCallback(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg);
  void PublisherThread();

protected:
  boost::mutex mutex_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::ServiceServer clear_buffer_srv_;
  std::thread pub_thread_;
  bool latch_;
  bool initialized_;
  bool queue_cleared_;
  double delay_;
  std::queue<StampedMessage> queue_;
};

} // namespace topic_buffer

#endif // TOPIC_BUFFER_NODELET_H