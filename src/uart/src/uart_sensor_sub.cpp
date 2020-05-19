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
        UartSensorListener::_AccX = half((float) msg->linear_acceleration.x);
        UartSensorListener::_AccY = half((float) msg->linear_acceleration.y);
        UartSensorListener::_AccZ = half((float) msg->linear_acceleration.z);

        UartSensorListener::_GyroX = half((float) msg->angular_velocity.x);
        UartSensorListener::_GyroY = half((float) msg->angular_velocity.y);
        UartSensorListener::_GyroZ = half((float) msg->angular_velocity.z);

        /*UartSensorListener::GetHalfBits(UartSensorListener::_AccX, sizeof(UartSensorListener::AccXPack), UartSensorListener::AccXPack);
        UartSensorListener::GetHalfBits(UartSensorListener::_AccY, sizeof(UartSensorListener::AccYPack), UartSensorListener::AccYPack);
        UartSensorListener::GetHalfBits(UartSensorListener::_AccZ, sizeof(UartSensorListener::AccZPack), UartSensorListener::AccZPack);

        UartSensorListener::GetHalfBits(UartSensorListener::_GyroX, sizeof(UartSensorListener::GyroXPack), UartSensorListener::GyroXPack);
        UartSensorListener::GetHalfBits(UartSensorListener::_GyroY, sizeof(UartSensorListener::GyroYPack), UartSensorListener::GyroYPack);
        UartSensorListener::GetHalfBits(UartSensorListener::_GyroZ, sizeof(UartSensorListener::GyroZPack), UartSensorListener::GyroZPack);*/

        UartSensorListener::SensorPubStatus = UartSensorListener::SensorPubStatus | 0x80;
    }

    void UartSensorListener::MagCb(const sensor_msgs::MagneticField::ConstPtr &msg)
    {
        UartSensorListener::_MagX = half((float) msg->magnetic_field.x);
        UartSensorListener::_MagY = half((float) msg->magnetic_field.y);
        UartSensorListener::_MagZ = half((float) msg->magnetic_field.z);

        /*UartSensorListener::GetHalfBits(UartSensorListener::_MagX, sizeof(UartSensorListener::MagXPack), UartSensorListener::MagXPack);
        UartSensorListener::GetHalfBits(UartSensorListener::_MagY, sizeof(UartSensorListener::MagYPack), UartSensorListener::MagYPack);
        UartSensorListener::GetHalfBits(UartSensorListener::_MagZ, sizeof(UartSensorListener::MagZPack), UartSensorListener::MagZPack);*/

        UartSensorListener::SensorPubStatus = UartSensorListener::SensorPubStatus | 0x40;
        

    }
}
