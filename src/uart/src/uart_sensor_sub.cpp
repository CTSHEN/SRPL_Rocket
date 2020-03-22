#include <uart_sensor_sub.h>

using half_float::half;

namespace uart_comm
{
    UartSensorListener::UartSensorListener()
    {
        // Create a subscriber.
        ImuSub_ = nh.subscribe("/imu",1, &UartSensorListener::ImuCb, this);
        MagSub_ = nh.subscribe("/magneticField", 1, &UartSensorListener::MagCb, this);
        // PixySub_ = 
        
    }

    // Callback Functions
    void UartSensorListener::ImuCb(const sensor_msgs::Imu::ConstPtr &msg)
    {
        _AccX = half((float) msg->linear_acceleration.x);
        _AccY = half((float) msg->linear_acceleration.y);
        _AccZ = half((float) msg->linear_acceleration.z);

        _GyroX = half((float) msg->angular_velocity.x);
        _GyroY = half((float) msg->angular_velocity.y);
        _GyroZ = half((float) msg->angular_velocity.z);

        //_ImuTimeStamp.data = 
    }

    void UartSensorListener::MagCb(const sensor_msgs::MagneticField::ConstPtr &msg)
    {
        _MagX = half((float) msg->magnetic_field.x);
        _MagY = half((float) msg->magnetic_field.y);
        _MagZ = half((float) msg->magnetic_field.z);

    }
}
