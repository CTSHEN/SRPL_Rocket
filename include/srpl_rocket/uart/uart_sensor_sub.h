/***************************************************
 * Uart comm packet definition
 * 
 * |offsets | bytes | Description|
 * |    6   |   2   | AccX|
 * |    8   |   2   | AccY|
 * |    10  |   2   | AccZ|
 * |    12  |   2   | GyroX|
 * |    14  |   2   | GyroY|
 * |    16  |   2   | GyroZ|
 * |    18  |   2   | MagX|
 * |    20  |   2   | MagY|
 * |    22  |   2   | MagZ|
 * |    24  |   2   | u1|
 * |    26  |   2   | v1|
 * |    28  |   2   | u2|
 * |    30  |   2   | v2|
 * |    32  |   2   | u3|
 * |    34  |   2   | v3|
 * |    36  |   1   | GPS states|
 * |    37  |   2   | GPS Altitude|
 * |    39  |   2   | GPS Longtitude|
 * |    41  |   2   | GPS Latitude|
 * |    43  |   2   | Quat 1|
 * |    45  |   2   | Quat 2|
 * |    47  |   2   | Quat 3|
 * |    49  |   2   | Quat 4|
 * |    51  |   2   | OmegaX|
 * |    53  |   2   | OmegaY|
 * |    55  |   2   | OmegaZ|
 * |    57  |   2   | PosX|
 * |    59  |   2   | PosY|
 * |    61  |   2   | PosZ|
 * |    63  |   2   | VelX|
 * |    65  |   2   | VelY|
 * |    67  |   2   | VelZ|
 * 
 *  
 * */

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

            half AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY,MagZ, \
            u1, u2, u3, v1, v2, v3, GPSAlt, GPSLong, GPSLat, Quat1, Quat2, Quat3, Quat4,\
            wX, wY, wZ, PosX, PosY, PosZ, VelX, VelY, VelZ;

           


            

            char AccXPack[2], AccYPack[2], AccZPack[2], GyroXPack[2], GyroYPack[2], GyroZPack[2],\
             MagXPack[2], MagYPack[2], MagZPack[2], u1Pack[2], u2Pack[2], u3Pack[2], v1Pack[2], v2Pack[2], v3Pack[2],\
             GPSAltPack[2], GPSLongPack[2], GPSLatPack[2], Quat1Pack[2], Quat2Pack[2], Quat3Pack[2], Quat4Pack[2],\
             wXPack[2], wYPack[2], wZPack[2], PosXPack[2], PosYPack[2], PosZPack[2], VelXPack[2], VelYPack[2], VelZPack[2];

            uint8_t GPS_Status;

                    

            //uint8_t Pack[87]; // Total 43*16-bit + 1*8-bit
            //uint8_t Pack[6]; // 3 16-bit data for testing
            /*char Pack1[2]; // Send 2 byte for testing
            char Pack2[2];
            char Pack3[2];*/

            void GetHalfBits(half f, size_t PackSize, char *Pack){
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
