#include <uart_sensor_sub.h>

using half_float::half;

namespace uart_comm
{
    UartSensorListener::UartSensorListener()
    {
        // Create a subscriber.
        ImuSub_ = nh.subscribe("/imu",1, &UartSensorListener::ImuCb, this);
        MagSub_ = nh.subscribe("/magnetometer", 1, &UartSensorListener::MagCb, this);
        // PixySub_ = 
        
    }

    // Callback Functions
    void UartSensorListener::ImuCb(const sensor_msgs::Imu::ConstPtr &msg)
    {
        UartSensorListener::AccX = half((float) msg->linear_acceleration.x);
        UartSensorListener::AccY = half((float) msg->linear_acceleration.y);
        UartSensorListener::AccZ = half((float) msg->linear_acceleration.z);

        UartSensorListener::GyroX = half((float) msg->angular_velocity.x);
        UartSensorListener::GyroY = half((float) msg->angular_velocity.y);
        UartSensorListener::GyroZ = half((float) msg->angular_velocity.z);

        

        UartSensorListener::SensorPubStatus = UartSensorListener::SensorPubStatus | 0x80;
    }

    void UartSensorListener::MagCb(const sensor_msgs::MagneticField::ConstPtr &msg)
    {
        UartSensorListener::MagX = half((float) msg->magnetic_field.x);
        UartSensorListener::MagY = half((float) msg->magnetic_field.y);
        UartSensorListener::MagZ = half((float) msg->magnetic_field.z);

        /*UartSensorListener::GetHalfBits(UartSensorListener::_MagX, sizeof(UartSensorListener::MagXPack), UartSensorListener::MagXPack);
        UartSensorListener::GetHalfBits(UartSensorListener::_MagY, sizeof(UartSensorListener::MagYPack), UartSensorListener::MagYPack);
        UartSensorListener::GetHalfBits(UartSensorListener::_MagZ, sizeof(UartSensorListener::MagZPack), UartSensorListener::MagZPack);*/

        UartSensorListener::SensorPubStatus = UartSensorListener::SensorPubStatus | 0x40;
        

    }
}
