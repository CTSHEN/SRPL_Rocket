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

#include<uart_sensor_sub.h> // ROS subscriber

using namespace std;
using half_float::half;

//Define Constants
const char *uart_target = "/dev/ttyTHS1";
#define     VMINX          1
#define     BAUDRATE       B921600

int main(int argc, char **argv)
{
    // Setup ROS
    ros::init(argc, argv, "uart_comm");

    

    ROS_INFO("UART Communication Node Start");
    // Create new uart_comm::UartSensorListener object
    uart_comm::UartSensorListener node;

    // Initialize some system status indecator
    node.SensorPubStatus = 0;

    ros::Rate loop_rate(1);

    // SETUP SERIAL WORLD
    int fid = -1;
    struct termios  port_options;   // Create the structure                          

    tcgetattr(fid, &port_options);	// Get the current attributes of the Serial port 

    	
    //------------------------------------------------
    //  OPEN THE UART
    //------------------------------------------------
    // The flags (defined in fcntl.h):
    //	Access modes (use 1 of these):
    //		O_RDONLY - Open for reading only.
    //		O_RDWR   - Open for reading and writing.
    //		O_WRONLY - Open for writing only.
    //	    O_NDELAY / O_NONBLOCK (same function) 
    //               - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
    //                 if there is no input immediately available (instead of blocking). Likewise, write requests can also return
    //				   immediately with a failure status if the output can't be written immediately.
    //                 Caution: VMIN and VTIME flags are ignored if O_NONBLOCK flag is set.
    //	    O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.fid = open("/dev/ttyTHS1", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode

    fid = open(uart_target, O_RDWR | O_NOCTTY );

    tcflush(fid, TCIFLUSH);
    tcflush(fid, TCIOFLUSH);
     	
    usleep(1000000);  // 1 sec delay

    if (fid == -1)
    {
        printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
    }
    	
    //------------------------------------------------
    // CONFIGURE THE UART
    //------------------------------------------------
    // flags defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html
    //	Baud rate:
    //         - B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, 
    //           B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, 
    //           B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
    //	CSIZE: - CS5, CS6, CS7, CS8
    //	CLOCAL - Ignore modem status lines
    //	CREAD  - Enable receiver
    //	IGNPAR = Ignore characters with parity errors
    //	ICRNL  - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
    //	PARENB - Parity enable
    //	PARODD - Odd parity (else even)

    port_options.c_cflag &= ~PARENB;            // Disables the Parity Enable bit(PARENB),So No Parity   
    port_options.c_cflag &= ~CSTOPB;            // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit 
    port_options.c_cflag &= ~CSIZE;	            // Clears the mask for setting the data size             
    port_options.c_cflag |=  CS8;               // Set the data bits = 8                                 	 
    port_options.c_cflag &= ~CRTSCTS;           // No Hardware flow Control                         
    port_options.c_cflag |=  CREAD | CLOCAL;                  // Enable receiver,Ignore Modem Control lines       				
    port_options.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both input & output
    port_options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode                            
    port_options.c_oflag &= ~OPOST;                           // No Output Processing

    port_options.c_lflag = 0;               //  enable raw input instead of canonical,
     		
    port_options.c_cc[VMIN]  = VMINX;       // Read at least 1 character
    port_options.c_cc[VTIME] = 0;           // Wait indefinetly 
    		
    cfsetispeed(&port_options,BAUDRATE);    // Set Read  Speed 
    cfsetospeed(&port_options,BAUDRATE);    // Set Write Speed 

    // Set the attributes to the termios structure
    int att = tcsetattr(fid, TCSANOW, &port_options);

    if (att != 0 )
    {
        ROS_INFO("\nERROR in Setting port attributes");
    }
    else
    {
        ROS_INFO("\nSERIAL Port Good to Go.\n");
    }

    // Flush Buffers
    tcflush(fid, TCIFLUSH);
    tcflush(fid, TCIOFLUSH);

    usleep(500000);   // 0.5 sec delay
    
    node.test3 = 4.5;
    node.GetHalfBits(node.test3, sizeof(node.Pack3), node.Pack3);
    
    // Wait until data arrived
    /*while(node.SensorPubStatus != 0xC0) // C0: IMU and Mag(testing), 0xFE for all sesnor
    {
        printf("SensorPubStatus: %x", node.SensorPubStatus);
        sleep(1); //sleep 1 sec waiting for all sensor data is ready
    }
    printf("SensorPubStatus: %x", node.SensorPubStatus);*/
    
    while(ros::ok())
    {
        node.GetHalfBits(node._AccX, sizeof(node.AccXPack), node.AccXPack);
        node.GetHalfBits(node._AccY, sizeof(node.AccYPack), node.AccYPack);
        node.GetHalfBits(node._AccZ, sizeof(node.AccZPack), node.AccZPack);
        //printf("AccX %x%x \n", node.AccZPack[1], node.AccZPack[0]);

        node.GetHalfBits(node._GyroX, sizeof(node.GyroXPack), node.GyroXPack);
        node.GetHalfBits(node._GyroY, sizeof(node.GyroYPack), node.GyroYPack);
        node.GetHalfBits(node._GyroZ, sizeof(node.GyroZPack), node.GyroZPack);

        node.GetHalfBits(node._MagX, sizeof(node.MagXPack), node.MagXPack);
        node.GetHalfBits(node._MagY, sizeof(node.MagYPack), node.MagYPack);
        node.GetHalfBits(node._MagZ, sizeof(node.MagZPack), node.MagZPack);
        
        //--------------------------------------------------------------
        // TRANSMITTING BYTES
        //--------------------------------------------------------------
        unsigned char tx_buffer[18]; // Accel, Gyro and Mag 36
        unsigned char *p_tx_buffer;
    	
        p_tx_buffer = &tx_buffer[0];
        
        *p_tx_buffer++ = node.AccXPack[1];
        *p_tx_buffer++ = node.AccXPack[0];
        *p_tx_buffer++ = node.AccYPack[1];
        *p_tx_buffer++ = node.AccYPack[0];
        *p_tx_buffer++ = node.AccZPack[1];
        *p_tx_buffer++ = node.AccZPack[0];
        //------------------------------------//
        *p_tx_buffer++ = node.GyroXPack[1];
        *p_tx_buffer++ = node.GyroXPack[0];
        *p_tx_buffer++ = node.GyroYPack[1];
        *p_tx_buffer++ = node.GyroYPack[0];
        *p_tx_buffer++ = node.GyroZPack[1];
        *p_tx_buffer++ = node.GyroZPack[0];
        //------------------------------------//
        *p_tx_buffer++ = node.MagXPack[1];
        *p_tx_buffer++ = node.MagXPack[0];
        *p_tx_buffer++ = node.MagYPack[1];
        *p_tx_buffer++ = node.MagYPack[0];
        *p_tx_buffer++ = node.MagZPack[1];
        *p_tx_buffer++ = node.MagZPack[0];
        

        //printf("ACCXPACK = %x%x", node.AccXPack[1], node.AccXPack[0]);


        /* *p_tx_buffer++ = node._AccX.data_byte[0];
        *p_tx_buffer++ = node._AccX.data_byte[1];
        *p_tx_buffer++ = node._AccX.data_byte[2];
        *p_tx_buffer++ = node._AccX.data_byte[3];
        *p_tx_buffer++ = node._AccY.data_byte[0];
        *p_tx_buffer++ = node._AccY.data_byte[1];
        *p_tx_buffer++ = node._AccY.data_byte[2];
        *p_tx_buffer++ = node._AccY.data_byte[3];
        *p_tx_buffer++ = node._AccZ.data_byte[0];
        *p_tx_buffer++ = node._AccZ.data_byte[1];
        *p_tx_buffer++ = node._AccZ.data_byte[2];
        *p_tx_buffer++ = node._AccZ.data_byte[3];
        *p_tx_buffer++ = node._GyroX.data_byte[0];
        *p_tx_buffer++ = node._GyroX.data_byte[1];
        *p_tx_buffer++ = node._GyroX.data_byte[2];
        *p_tx_buffer++ = node._GyroX.data_byte[3];
        *p_tx_buffer++ = node._GyroY.data_byte[0];
        *p_tx_buffer++ = node._GyroY.data_byte[1];
        *p_tx_buffer++ = node._GyroY.data_byte[2];
        *p_tx_buffer++ = node._GyroY.data_byte[3];
        *p_tx_buffer++ = node._GyroZ.data_byte[0];
        *p_tx_buffer++ = node._GyroZ.data_byte[1];
        *p_tx_buffer++ = node._GyroZ.data_byte[2];
        *p_tx_buffer++ = node._GyroZ.data_byte[3];
        *p_tx_buffer++ = node._MagX.data_byte[0];
        *p_tx_buffer++ = node._MagX.data_byte[1];
        *p_tx_buffer++ = node._MagX.data_byte[2];
        *p_tx_buffer++ = node._MagX.data_byte[3];
        *p_tx_buffer++ = node._MagY.data_byte[0];
        *p_tx_buffer++ = node._MagY.data_byte[1];
        *p_tx_buffer++ = node._MagY.data_byte[2];
        *p_tx_buffer++ = node._MagY.data_byte[3];
        *p_tx_buffer++ = node._MagZ.data_byte[0];
        *p_tx_buffer++ = node._MagZ.data_byte[1];
        *p_tx_buffer++ = node._MagZ.data_byte[2];
        *p_tx_buffer++ = node._MagZ.data_byte[3];*/



	




        //printf("fid 1=%d\n", fid );
    	
        if (fid != -1)
        {
    	    int count = write(fid, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));		//Filestream, bytes to write, number of bytes to write

            usleep(1000);   // .001 sec delay

        //printf("Count = %d\n", count);

    	    if (count < 0)  ROS_INFO("UART TX error\n");
        }

        //usleep(1000000);  // 1 sec delay
        ros::spinOnce();
        loop_rate.sleep();

    }
    




}


