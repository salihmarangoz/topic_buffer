#include <pluginlib/class_list_macros.h>
#include <topic_buffer/topic_buffer_nodelet.h>


namespace topic_buffer
{

TopicBufferNodelet::TopicBufferNodelet(){}

TopicBufferNodelet::~TopicBufferNodelet()
{
  pub_thread_.join();
}

void 
TopicBufferNodelet::onInit()
{
  pnh_ = getPrivateNodeHandle();
  pnh_.param("delay", delay_, 1.0);

  //pnh_.param("latch", latch_, false);
  latch_ = false; // TODO: If I am not wrong the output queue need to be at least 2 for latching. Just leaving this feature disabled.

  initialized_ = false;
  queue_cleared_ = false;

  clear_buffer_srv_ = pnh_.advertiseService("clear_buffer", &TopicBufferNodelet::clearBufferCallback, this);
  sub_ = pnh_.subscribe<topic_tools::ShapeShifter>("input", 1, &TopicBufferNodelet::messageCallback, this);
}

ros::Publisher 
TopicBufferNodelet::advertise(boost::shared_ptr<topic_tools::ShapeShifter const> msg,const std::string& topic)
{
  ros::AdvertiseOptions opts(topic, 1, msg->getMD5Sum(), msg->getDataType(), msg->getMessageDefinition());
  opts.latch = latch_;
  return pnh_.advertise(opts);
}

bool TopicBufferNodelet::clearBufferCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  NODELET_INFO("Cleared topic buffer!");
  mutex_.lock();
  queue_ = {}; // clear queue
  queue_cleared_ = true;
  mutex_.unlock();
  return true;
}

void
TopicBufferNodelet::messageCallback(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg)
{
  if (!initialized_)
  {
    pub_ = advertise(msg, "output");
    initialized_ = true;
    pub_thread_ = std::thread(&TopicBufferNodelet::PublisherThread, this);
  }

  mutex_.lock();
  auto publish_time = std::chrono::system_clock::now() + std::chrono::milliseconds(uint64_t(1000*delay_));
  queue_.push(StampedMessage(msg, publish_time));
  mutex_.unlock();
}

void 
TopicBufferNodelet::PublisherThread()
{
  while (ros::ok())
  {
    while (queue_.size() > 0) // cache the latest message
    {
      mutex_.lock();
      auto current_time = std::chrono::system_clock::now();

      if (current_time < queue_.front().publish_time)
      {
        mutex_.unlock();
        std::this_thread::sleep_for(queue_.front().publish_time - current_time); // wait for the front message's time to be published
        mutex_.lock();
      }

      if (!queue_cleared_) // if queue is cleared, don't publish the cached message
      {
        pub_.publish(queue_.front().msg);
        queue_.pop();
      }
      queue_cleared_ = false;

      mutex_.unlock();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(uint64_t(500*delay_)));
  }
}

} // namespace topic_buffer

PLUGINLIB_EXPORT_CLASS(topic_buffer::TopicBufferNodelet, nodelet::Nodelet)