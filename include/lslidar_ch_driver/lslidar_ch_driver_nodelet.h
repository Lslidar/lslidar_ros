

#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <lslidar_ch_driver/lslidar_ch_driver.h>

namespace lslidar_ch_driver
{

class LslidarChDriverNodelet: public nodelet::Nodelet
{
public:

  LslidarChDriverNodelet();
  ~LslidarChDriverNodelet();

private:

  virtual void onInit(void);
  virtual void devicePoll(void);

  volatile bool running;               ///< device thread is running
  boost::shared_ptr<boost::thread> device_thread;

  LslidarChDriverPtr lslidar_ch_driver; ///< driver implementation class
};

} // namespace lslidar_driver
