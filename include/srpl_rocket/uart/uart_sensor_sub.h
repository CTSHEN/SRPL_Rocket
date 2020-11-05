#ifndef UART_SENSOR_SUB_H
#define UART_SENSOR_SUB_H

// Ros includes.
#include <ros/ros.h>
#include <ros/time.h>

// sensor data lib
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <pixy_msgs/PixyData.h>
#include <pixy_msgs/PixyBlock.h>

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

            //void Pixy0Cb(const pixy_msgs::PixyData::ConstPtr &msg);
            //void Pixy120Cb(const pixy_msgs::PixyData::ConstPtr &msg);
            //void Pixy240Cb(const pixy_msgs::PixyData::ConstPtr &msg); //TODO change to float64MultiArray
            
            //TODO Need to add a new Cb for GPS***

            //void PoseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
            //void TwistCb(const geometry_msgs::TwistWithCovarianceStamped::Constptr &msg);

            

            uint8_t SensorPubStatus;  // 8-bits: | IMU | MAG | GPS1 | GPS2 | PIXY1 | PIXY2 | PIXY3 | 0 |
            bool StateEstStatus;

            half _AccX, _AccY, _AccZ, _GyroX, _GyroY, _GyroZ, _MagX, _MagY, _MagZ;

            char AccXPack[2], AccYPack[2], AccZPack[2], GyroXPack[2], GyroYPack[2], GyroZPack[2], MagXPack[2], MagYPack[2], MagZPack[2];

            half test3;
            char Pack3[2]; //for testing



            

            //uint8_t Pack[87]; // Total 43*16-bit + 1*8-bit
            //uint8_t Pack[6]; // 3 16-bit data for testing
            /*char Pack1[2]; // Send 2 byte for testing
            char Pack2[2];
            char Pack3[2];*/

            void GetHalfBits(half f, size_t PackSize, char *Pack){
                //printf("sizeof Pack is %d\n", PackSize);
                //printf("size of f is %d\n", sizeof(f));
                assert(sizeof(f) == PackSize);
                memcpy(Pack,&f,sizeof(f));   
            }


			

        
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
