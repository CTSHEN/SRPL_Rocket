/************************************************
 * uart_comm_node
 * Description:
 * This program subscribes sensor data, fused states (position, attitude information)
 * packs these data into a data string and output through the uart port of
 * Nvidia Jetson Nano in 10 Hz.
 * The reference of UART communication code if from
 * https://devtalk.nvidia.com/default/topic/1057441/jetson-nano/jetson-nano-uart-c-c-example/2
 *  
 * Subscribe:
 * 1. \imu
 * 2. \magneticField
 * 3. \poseFused
 * 4. \gps
 * 
 * Output:
 * 
 * 
 * *********************************************/

#include <stdio.h>
#include <unistd.h>       // Used for UART
#include <sys/fcntl.h>    // Used for UART
#include <termios.h>      // Used for UART
#include <string>

using namespace std;

//Define Constants
const char *uart_target = "/dev/ttyTHS1";
#define     VMINX          1
#define     BAUDRATE       B115200