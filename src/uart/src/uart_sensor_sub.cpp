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
        _AccX.data = half((float) msg->linear_acceleration.x);
        _AccY.data = half((float) msg->linear_acceleration.y);
        _AccZ.data = half((float) msg->linear_acceleration.z);

        _GyroX.data = half((float) msg->angular_velocity.x);
        _GyroY.data = half((float) msg->angular_velocity.y);
        _GyroZ.data = half((float) msg->angular_velocity.z);

        //_ImuTimeStamp.data = 
    }

    void UartSensorListener::MagCb(const sensor_msgs::MagneticField::ConstPtr &msg)
    {
        _MagX.data = half((float) msg->magnetic_field.x);
        _MagY.data = half((float) msg->magnetic_field.y);
        _MagZ.data = half((float) msg->magnetic_field.z);

    }
}
