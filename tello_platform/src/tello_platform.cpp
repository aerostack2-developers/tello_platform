
/*!*******************************************************************************************
 *  \file       tello_platform.cpp
 *  \brief      Implements the functionality and communication with the tello drone.
 *  \authors    Daniel Fernández Sánchez
 *              Pedro Arias Pérez
 *              Miguel Fernández Cortizas
 *              
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include "tello_platform.hpp"


TelloPlatform::TelloPlatform():as2::AerialPlatform(){
  connected = false;
  tello = new Tello;
  connected = true;
  /*motionRecord.push_back(referencePoint);

  if (motionRecord[motionRecord.end()].isApprox(referencePoint)){
    cout<<"funciona"<<endl;
  }*/
  referencePoint.push_back(0.0); // x
  referencePoint.push_back(0.0); // y
  referencePoint.push_back(0.0); // z
  referencePoint.push_back(0.0); // yaw

  motionRecord.push_back(referencePoint);
  referenceSpeed = {0.0,0.0,0.0,0.0};
  this->declare_parameter<double>("minSpeed", 0.02);
  this->declare_parameter<double>("maxSpeed", 15.0);
  configureSensors();
  //as2_msgs::msg::PlatformInfo::connected = true;
  //as2_msgs::msg::PlatformInfo::armed = true;
  
  //imu_callback = std::bind(&TelloPlatform::recvIMU, this);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10),[this](){recvIMU();
                                           recvBattery();
                                           recvBarometer();
                                           this->sendCommand();});

}

TelloPlatform::~TelloPlatform(){
  tello->~Tello();
  delete (tello);
}

// *********************************************************
// ***************** Aerial Platform Methods ***************
// *********************************************************
// Configure sensors creating th publishers to the topics. 
void TelloPlatform::configureSensors(){
  // IMU
  imu_sensor_ptr = std::make_unique<as2::sensors::Imu>("imu", this);
  // ODOMETRY
  // De donde saco la odometria
  odometry_ptr = std::make_unique<as2::sensors::Sensor<nav_msgs::msg::Odometry>>("odometry", this);
  // BATTERY
  battery_ptr = std::make_unique<as2::sensors::Battery>("battery", this);
  // IMAGE
  //image_ptr = std::make_unique<as2::sensors::Camera>("camera", this);
  barometer_ptr = std::make_unique<as2::sensors::Barometer>("barometer", this);
}
// It sends command of motion. 
// It needs to recognise the control mode. 
bool TelloPlatform::ownSendCommand(){
  bool success;
  pair<bool, string> response; 
  as2_msgs::msg::ControlMode controlmode = this->getControlMode();

  double x,y,z,yaw;
  if (ownSetArmingState(true) && controlmode.reference_frame == as2_msgs::msg::ControlMode::BODY_FLU_FRAME){
    if (controlmode.control_mode == as2_msgs::msg::ControlMode::POSITION
        && controlmode.yaw_mode == as2_msgs::msg::ControlMode::YAW_ANGLE){
      double limitPos[2]= {20, 500};
      double x_m,y_m,z_m,yaw_rad;
      x_m = this->command_pose_msg_.pose.position.x; // en m
      y_m = this->command_pose_msg_.pose.position.y; // en m
      z_m = this->command_pose_msg_.pose.position.z; // en m
      yaw_rad = (this->command_pose_msg_.pose.orientation.z);
      /*Vector4d ref(x_m, y_m, z_m, yaw_rad);
      if (!motionRecord.empty()){
        Vector4d aux = motionRecord[motionRecord.end()];
      }*/
      if (!motionRecord.empty()){
        vector<double> lastOrder_rec = motionRecord.back();
        vector<double> lastOrder = {x_m, y_m, z_m, yaw_rad};
        if (lastOrder_rec != lastOrder){
            //cout<<"lastOrder_rec: "<<lastOrder_rec[0]<<", "<<lastOrder_rec[1]<<", "<<lastOrder_rec[2]<<", "<<lastOrder_rec[3]<<endl;
            //cout<<"lastOrder: "<<lastOrder[0]<<", "<<lastOrder[1]<<", "<<lastOrder[2]<<", "<<lastOrder[3]<<endl;
            vector<double> newReference;
            double aux_subs;
            for (int comp = 0; comp<4; comp++){
              aux_subs = lastOrder[comp]-lastOrder_rec[comp];
              if (abs(aux_subs)<0.0002)
                aux_subs = 0.0;
              newReference.push_back(aux_subs);
            }
            //cout<<"newReference: "<<newReference[0]<<", "<<newReference[1]<<", "<<newReference[2]<<", "<<newReference[3]<<endl;
            x = newReference[0]*100; // en cm
            y = newReference[1]*100; // en cm
            z = newReference[2]*100; // en cm
            yaw = newReference[3]*180/M_PI;
            //cout<<"Send it: "<<x<<", "<<y<<", "<<z<<", "<<yaw<<endl;
            x = filterLimits(x,limitPos[0], limitPos[1], "X position");
            y = filterLimits(y,limitPos[0], limitPos[1], "Y position");
            z = filterLimits(z,limitPos[0], limitPos[1], "Z position");
            yaw = filterLimits(yaw, 1, 360, "Yaw orientation");

            motionRecord.push_back(lastOrder);
            
            bool xSend = tello->x_motion(x);
            bool ySend = tello->y_motion(y);
            bool zSend = tello->z_motion(z);
            bool yawSend = tello->yaw_twist(yaw);

            if (!xSend){
              RCLCPP_INFO(this->get_logger(), "Tello Platform: Error sending X position command");
              return false;
            }else if(!ySend){
              RCLCPP_INFO(this->get_logger(), "Tello Platform: Error sending Y position command");
              return false;
            }else if (!zSend){
              RCLCPP_INFO(this->get_logger(), "Tello Platform: Error sending Z position command");
              return false;
            }else if(!yawSend){
              RCLCPP_INFO(this->get_logger(), "Tello Platform: Error sending Yaw orientation command");
              return false;
            }
        }
      }
      

      
    }
    else if (controlmode.control_mode == as2_msgs::msg::ControlMode::SPEED
             && controlmode.yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED){
      double limitSpeed[2]= {-100, 100};
      double x_, y_, z_, yaw_;
      rclcpp::Parameter minSpeed_param = this->get_parameter("minSpeed");
      rclcpp::Parameter maxSpeed_param = this->get_parameter("maxSpeed");

      double minSpeed = minSpeed_param.as_double();
      double maxSpeed = maxSpeed_param.as_double();
      
      // m/s
      x_ = this->command_twist_msg_.twist.linear.x; 
      y_ = this->command_twist_msg_.twist.linear.y;
      z_ = this->command_twist_msg_.twist.linear.z;
      // rad/s
      yaw_ = this->command_twist_msg_.twist.angular.z;
      vector<double> newMsgSpeed = {x_, y_, z_, yaw_};
      //cout<<"Receive: "<<x_<<", "<<y_<<", "<<z_<<", "<<yaw_<<endl;
      if (referenceSpeed != newMsgSpeed){ 
        x = filterLimitsSpeed(x_, minSpeed, maxSpeed, "X speed");
        y = filterLimitsSpeed(y_,minSpeed, maxSpeed, "Y speed");
        z = filterLimitsSpeed(z_,minSpeed, maxSpeed, "Z speed");
        yaw = filterLimitsSpeed(yaw_, minSpeed, maxSpeed, "Yaw angular");
        //cout<<"Filter: "<<x<<", "<<y<<", "<<z<<", "<<yaw<<endl;
        
        x_ = (double(100)/(maxSpeed-minSpeed))*(abs(x)-minSpeed);
        y_ = (double(100)/(maxSpeed-minSpeed))*(abs(y)-minSpeed);
        z_ = (double(100)/(maxSpeed-minSpeed))*(abs(z)-minSpeed);
        yaw_ = (double(100)/(maxSpeed-minSpeed))*(abs(yaw)-minSpeed);
        //cout<<"Interp: "<<x_<<", "<<y_<<", "<<z_<<", "<<yaw_<<endl;
        x = x==0 ? 0 : x/abs(x)*x_;
        y = y==0 ? 0 : y/abs(y)*y_;
        z = z==0 ? 0 : z/abs(z)*z_;
        yaw = yaw==0 ? 0: yaw/abs(yaw)*yaw_;
        //cout<<"Send: "<<x<<", "<<y<<", "<<z<<", "<<yaw<<endl;
        bool speedSend = tello->speedMotion(x, y, z, yaw);
        
        if (!speedSend){
            RCLCPP_INFO(this->get_logger(), "Tello Platform: Error sending control speed command");
            return false;
        }
        //cout << newMsgSpeed[0] << " "<<newMsgSpeed[1]<<" "<<newMsgSpeed[2]<< " "<<newMsgSpeed[3]<<endl;
        referenceSpeed = newMsgSpeed;
      }
    }
    
  }

  return true;
}
// Set arming state
bool TelloPlatform::ownSetArmingState(bool state){
  pair<bool, string> resp;
  bool ret = false;
  if (state){
      if (!connected){
        resp = tello->sendCommand("command");
        if (resp.first == true and resp.second!="Error"){
          ret = true;
        }
      }else{
        ret = true;
      }
  }
  return ret;
}
// set offboard control
bool TelloPlatform::ownSetOffboardControl(bool offboard){
  if (connected){
    return true;}
  else{
    return false;
  }
}
// Check if the control mode message is avilable in tello platform. 
bool TelloPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode& msg){
    bool valid_mode = false;
    if (msg.reference_frame == as2_msgs::msg::ControlMode::BODY_FLU_FRAME){
      if (msg.control_mode==as2_msgs::msg::ControlMode::POSITION &&
          msg.yaw_mode == as2_msgs::msg::ControlMode::YAW_ANGLE){
            RCLCPP_INFO(this->get_logger(), "Position mode enabled");
            valid_mode = true;

          }else if (msg.control_mode==as2_msgs::msg::ControlMode::SPEED &&
          msg.yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED) {
            RCLCPP_INFO(this->get_logger(), "Speed mode enabled");
            valid_mode = true;
          }
      else if (msg.control_mode == as2_msgs::msg::ControlMode::UNSET){
        valid_mode = true;
      }
    }
    
    return valid_mode;
}
// Order to platform that takes off 
bool TelloPlatform::ownTakeoff(){ 
  if (connected){
    tello->sendCommand("takeoff");
    //sleep(0.5);
    double height = tello->getHeight();
    vector<double> take_off = {0,0,height,0};
    motionRecord.push_back(take_off);
    return true;
  }else{
    return false;
  }
}
// Order to platform that lands 
bool TelloPlatform::ownLand(){
  if (connected){
    tello->sendCommand("land");

    return true;
  }else{
    return false;
  }
}

// **********************************************************
// ******************** CALLBACK METHODS ********************
// **********************************************************
// Receive the IMU data and publish it into its topic.
void TelloPlatform::recvIMU(){
  auto timestamp = this->get_clock()->now();
  vector<coordinates> imu_info =  tello->getIMU();
  
  tf2::Quaternion q;
  float roll_rad = float(imu_info[0].x*M_PI)/180;
  float pitch_rad = float(imu_info[0].y*M_PI)/180;
  float yaw_rad = float(imu_info[0].z*M_PI)/180;
  q.setRPY(roll_rad, pitch_rad, yaw_rad);

  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = timestamp;
  imu_msg.header.frame_id = "imu";

  imu_msg.orientation.x = q.x();
  imu_msg.orientation.y = q.y();
  imu_msg.orientation.z = q.z();
  imu_msg.orientation.w = q.w();
  imu_msg.angular_velocity.x = imu_info[1].x;
  imu_msg.angular_velocity.y = imu_info[1].y;
  imu_msg.angular_velocity.z = imu_info[1].z;

  imu_msg.linear_acceleration.x = imu_info[2].x;
  imu_msg.linear_acceleration.y = imu_info[2].y;
  imu_msg.linear_acceleration.z = imu_info[2].z;
  
  imu_sensor_ptr->updateData(imu_msg);
}

// Receive the Battery percentage and publish it into its topic.
void TelloPlatform::recvBattery(){
  double bat = tello->getBattery();

  auto timestamp = this->get_clock()->now();

  sensor_msgs::msg::BatteryState battery_msg;
  battery_msg.header.stamp = timestamp;

  battery_msg.percentage = bat;

  battery_ptr->updateData(battery_msg);
}

// Receive the Barometer percentage and publish it into its topic.
void TelloPlatform::recvBarometer(){
  double baro = tello->getBarometer();
  auto timestamp = this->get_clock()->now();

  sensor_msgs::msg::FluidPressure barometer_msg;
  barometer_msg.header.stamp = timestamp;
  barometer_msg.fluid_pressure = baro;
  barometer_ptr-> updateData(barometer_msg);
}
// **********************************************************
// ******************** AUXILIAR METHODS ********************
// **********************************************************
double TelloPlatform::filterLimits (double value, double minValue, double maxValue, string info){
    double value_filter = value;
    if (value != 0 && abs(value)<minValue || abs(value)>maxValue){
        if (abs(value)<minValue){
          value_filter = (abs(value)/value)*minValue;
          RCLCPP_INFO(this->get_logger(),"%s command is set to its minimum value. %.1f", info, minValue);
        }
        else{
          value_filter = (abs(value)/value)*maxValue;
          RCLCPP_INFO(this->get_logger(), "%s command is set to its maximum value %.1f", info, maxValue);
        }
    }
    return value_filter;
}

double TelloPlatform::filterLimitsSpeed (double value, double minValue, double maxValue, string info){
    double value_filter = value;
    if (abs(value)<minValue || abs(value)>maxValue){
        if (abs(value)<minValue){
          if (value!=0){
            value_filter = (abs(value)/value)*minValue;
          }else{
            value_filter = 0;
          }
          RCLCPP_INFO(this->get_logger(),"%s command is set to its minimum value. %.2f", info, minValue);
        }
        else{
          if (value!=0){
            value_filter = (abs(value)/value)*maxValue;
          }else{
            value_filter = maxValue;
          }
          RCLCPP_INFO(this->get_logger(), "%s command is set to its maximum value %.2f", info, maxValue);
        }
    }
    return value_filter;
}
/*void TelloPlatform::recvVideo(){
  Mat frame = tello->getFrame();
  if (!frame.empty())
  {
      imshow("CTello Stream", frame);
  }

}*/
