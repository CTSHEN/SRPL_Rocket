#ifndef UART_SENSOR_SUB_H
#define UART_SENSOR_SUB_H

// Ros includes.
#include <ros/ros.h>
#include <ros/time.h>

// sensor data lib
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
//***[TBC] Need to include lib for pixy msgs***

// states data lib
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
namespace uart_comm
{
    class UartSensorListener
    {
        public:
            // constructor
            UartSensorListener(ros::NodeHandle nh);

            void ImuCb(const sensor_msgs::Imu::ConstPtr &msg );
            void MagCb(const sensor::msgs::MagneticField::ConstPtr &msg);
            //***[TBC] Need to add a new Cb for pixy***

            void PoseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
            void TwistCb(const geometry_msgs::TwistWithCovarianceStamped::Constptr &msg);

        
        private:
            // Subscriber for sensor data
            ros::Subscriber ImuSub_;
            ros::Subscriber MagSub_;
            ros::Subscriber PixySub_;
            //***[TBD] There are 3 pixy cameras, so there will be three subscribers***

            // Subcriber for state data
            ros::Subscriber PoseSub_;
            ros::Subscriber TwistSub_;
     };
}

#endif  //UART_SENSOR_SUB_H