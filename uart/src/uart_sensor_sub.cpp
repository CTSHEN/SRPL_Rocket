#include <uart_sensor_sub.h>

namespace uart_comm
{
    UartSensorListener::UartSensorListener(ros::NodeHande nh)
    {
        // Create a subscriber.
        ImuSub_ = nh.subscribe("/imu",1, &UartSensorListener::ImuCb, this);
        MagSub_ = nh.subscribe("/magneticField", 1, &UartSensorListener::MagCb, this);
        // PixySub_ = 
        
    }

    // Callback Functions
    void ImuCb(const sensor_msgs::Imu::ConstPtr &msg)
    {
        _AccX.data = (float) msg.linear_acceleration.x;
        _AccY.data = (float) msg.linear_acceleration.y;
        _AccZ.data = (float) msg.linear_acceleration.z;

        _GyroX.data = (float) msg.angular_velocity.x;
        _GyroY.data = (float) msg.angular_velocity.y;
        _GyroZ.data = (float) msg.angular_velocity.z;

        //_ImuTimeStamp.data = 
    }

    void MagCb(const snesor_msgs::MagneticField::ConstPtr &msg)
    {
        _MagX.data = (float) msg.magnetic_field.x;
        _MagY.data = (float) msg.magnetic_field.y;
        _MagZ.data = (float) msg.magnetic_field.z;

    }
}