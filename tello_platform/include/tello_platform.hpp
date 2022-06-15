#include <chrono>
#include <memory>
#include "tello.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "as2_core/node.hpp"
#include "as2_core/core_functions.hpp"


#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include "as2_core/aerial_platform.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <string.h>
#include <vector>
//#include "cv_bridge/CvBridge.h"

using Eigen::Vector4d;
using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace std;
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class TelloPlatform : public as2::AerialPlatform{
    private:
      Tello* tello;
      rclcpp::TimerBase::SharedPtr timer_;
      std::unique_ptr<as2::sensors::Imu> imu_sensor_ptr;
      std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> odometry_ptr;
      std::unique_ptr<as2::sensors::Battery> battery_ptr;
      std::unique_ptr<as2::sensors::Barometer> barometer_ptr;
      //std::unique_ptr<as2::sensors::Camera> image_ptr;
      bool connected;
      //Vector4d referencePoint(0,0,0,0); 
      //std::vector<Vector4d> motionRecord; 
      vector<double> referencePoint; 
      vector<vector<double>> motionRecord;
      vector<double> referenceSpeed;
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subs;
      void callback1 (const std_msgs::msg::String::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), msg->data.c_str());
      }

    private:
      void recvIMU();
      void recvBattery();
      void recvBarometer();
      double filterLimits (double value, double minValue, double maxValue, string info);
      double filterLimitsSpeed (double value, double minValue, double maxValue, string info);
      //void recvVideo();
    public:
      TelloPlatform();
      ~TelloPlatform();
      void configureSensors();
      bool ownSendCommand();
      bool ownSetArmingState(bool state);
      bool ownSetOffboardControl(bool offboard);
      bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode& msg);

      bool ownTakeoff();
      bool ownLand();

      
};