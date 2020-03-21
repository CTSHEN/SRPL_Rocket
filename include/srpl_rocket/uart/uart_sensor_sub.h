#ifndef UART_SENSOR_SUB_H
#define UART_SENSOR_SUB_H

// Ros includes.
#include <ros/ros.h>
#include <ros/time.h>

// sensor data lib
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
//***[TBC] Need to include lib for pixy msgs***
//***[TBC] Need to include lib for GPS msgs***

// states data lib
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

// half precision float lib
#include "half.hpp"

#include <boost/variant.hpp>

using half_float::half; 
namespace uart_comm
{
    class UartSensorListener
    {
        public:
			ros::NodeHandle nh;
            // constructor
            UartSensorListener();

            void ImuCb(const sensor_msgs::Imu::ConstPtr &msg );
            void MagCb(const sensor_msgs::MagneticField::ConstPtr &msg);
            //***[TBC] Need to add a new Cb for pixy***
            //***[TBC] Need to add a new Cb for GPS***

            //void PoseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
            //void TwistCb(const geometry_msgs::TwistWithCovarianceStamped::Constptr &msg);

            union DataTrans
            {
                float data;
                //half data;
                //unsigned char data_byte[4];
				uint8_t data_byte[2];
            };

            union TimeTrans
            {
                uint32_t time;
                unsigned char time_byte[4];
            };
            


			// IMU
            DataTrans _AccX;
            DataTrans _AccY;
            DataTrans _AccZ;
            DataTrans _GyroX;
            DataTrans _GyroY;
            DataTrans _GyroZ;
            TimeTrans _ImuTimeStamp;

            // Magnetometer
            DataTrans _MagX;
            DataTrans _MagY;
            DataTrans _MagZ;
            TimeTrans _MagTimeStamp;

            // Variables for Pixy and GPS [TBC]

            //States
            DataTrans _PosX;
            DataTrans _PosY;
            DataTrans _PosZ;
            DataTrans _TwistX;
            DataTrans _TwistY;
            DataTrans _TwistZ;
            DataTrans _Q1;
            DataTrans _Q2;
            DataTrans _Q3;
            DataTrans _Q4;
            DataTrans _OmegaX;
            DataTrans _OmegaY;
            DataTrans _OmegaZ;
            TimeTrans _StateTimeStamp;
            

        
        private:
			
            // Subscriber for sensor data
            ros::Subscriber ImuSub_;
            ros::Subscriber MagSub_;
           // ros::Subscriber PixySub_;
            //***[TBD] There are 3 pixy cameras, so there will be three subscribers***
           // ros::Subscriber GpsSub_;

            // Subcriber for state data
           // ros::Subscriber PoseSub_;
           // ros::Subscriber TwistSub_;

            






     };
}

#endif  //UART_SENSOR_SUB_H
