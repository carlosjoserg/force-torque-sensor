#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Empty.h>

#define LINUX
#include <Golem/SM/SMHelper.h>
#include <stdio.h>

#include <string>

// FTSensor class definition
// #include "FTSensor/FTSensor.h"

namespace ftsensor {

class FTSensorHW
{
  private:
    //! The node handle
    ros::NodeHandle nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    //! The sensor
    // FTSensor* ftsensor_;
    std::string ip_;
    std::string name_;
    std::string type_;
    unsigned id_;

    // GOLEM STUFF
    golem::SMTimer timer_;
    golem::SM::MessageStream msgstr_;
    golem::SMClient *client_ptr_;
    size_t capacity_;

    // measurements
    std::vector<double> ft_;
    std::vector<double> ft_bias_;

    //! Publisher for sensor readings
    ros::Publisher pub_sensor_readings_;

    //! Service for setting the bias
    ros::ServiceServer srv_set_bias_;

  public:
    //------------------ Callbacks -------------------
    // Callback for setting bias
    bool setBiasCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

    // Publish the measurements
    void publishMeasurements();

    //! Subscribes to and advertises topics
    FTSensorHW(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {

      priv_nh_.param<std::string>("name", name_, "my_sensor");
      priv_nh_.param<std::string>("type", type_, "nano17");
      priv_nh_.param<std::string>("ip", ip_, "192.168.0.51");

      ROS_INFO("FT Sensor config:");
      ROS_INFO_STREAM("ip: " << ip_);
      ROS_INFO_STREAM("name: " << name_);
      ROS_INFO_STREAM("type: " << type_);


      // Init FT Sensor
      capacity_ = 10000000;
      const std::string host = ip_;
      id_ = 0;
      const unsigned short port = (unsigned short)atoi("26873");
      client_ptr_ = new golem::SMClient(host, port, timer_, &msgstr_);
      client_ptr_->syncTimers();
      client_ptr_->start();

      // init data containers
      ft_.resize(6);
      ft_bias_.resize(6);

      // Advertise topic where readings are published
      pub_sensor_readings_ = nh_.advertise<geometry_msgs::WrenchStamped>(nh_.resolveName("sensor_measurements"), 10);

      // Advertise service for setting the bias
      srv_set_bias_ = nh_.advertiseService(nh_.resolveName("tare"), &FTSensorHW::setBiasCallback, this);
    }

    //! Empty stub
    ~FTSensorHW() {}

};

// ToDo: setBias and Tare are different things
// setBias(A B C D E F) to set an external bias manually
// Tare() it set the bias such that all sensor_measurements are zero
bool FTSensorHW::setBiasCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  ft_bias_ = ft_;

  return true;
}

void FTSensorHW::publishMeasurements()
{
  geometry_msgs::WrenchStamped ftreadings;

  std::vector<char> data(capacity_);
  std::vector<char> data_prev(capacity_);
  // float measurements[6];

  golem::SM::Header header(capacity_, id_);
  if (!client_ptr_->read(header, &data.front(), boost::posix_time::seconds(1)))
	  return;
  for(int i = 0; i < 6 ; i++){
	  char *buf = &data[i*8];
	  std::copy(buf, buf + sizeof(double), reinterpret_cast<char*>(&ft_[i]));
  }
  //std::cout << ft[0] << " " << ft[1] << " " << ft[2] << " " << ft[3] << " " << ft[4] << " " << ft[5] << "\n";

  ftreadings.wrench.force.x = ft_[0] - ft_bias_[0];
  ftreadings.wrench.force.y = ft_[1] - ft_bias_[1];
  ftreadings.wrench.force.z = ft_[2] - ft_bias_[2];
  ftreadings.wrench.torque.x = ft_[3] - ft_bias_[3];
  ftreadings.wrench.torque.y = ft_[4] - ft_bias_[4];
  ftreadings.wrench.torque.z = ft_[5] - ft_bias_[5];

  ftreadings.header.stamp = ros::Time::now();
  ftreadings.header.frame_id = name_ + "_" + type_ + "_" + "measure";

  pub_sensor_readings_.publish(ftreadings);
  return;
}

} // namespace ftsensor

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ft_sensor_hw");
  ros::NodeHandle nh;
  ros::Rate loop(100);
  ftsensor::FTSensorHW node(nh);

  while(ros::ok())
  {
    node.publishMeasurements();
    ros::spinOnce();

    loop.sleep();
  }
  return 0;
}
