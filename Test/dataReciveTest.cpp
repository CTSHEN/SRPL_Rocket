    #include <stdio.h>
    #include <unistd.h>       // Used for UART
    #include <sys/fcntl.h>    // Used for UART
    #include <termios.h>      // Used for UART
    #include <string>
    #include <math.h>
    #include "half.hpp"

    using namespace std;
    using half_float::half; 


    // Define Constants
    const char *uart_target = "/dev/ttyUSB0";
    #define     NSERIAL_CHAR   256
    #define     VMINX          1
    #define     BAUDRATE       B921600

    float changeToFloat(uint16_t data){
        bool S = (bool) (data >> 15);
        uint8_t E = -15 + ((data & (0b0111110000000000))>>10);
        float M;
        float value;
        M = 1+ ((data & (0b0000001000000000))>>9)*pow(2,-1)+ ((data & (0b0000000100000000))>>8)*pow(2,-2)+\
        ((data & (0b0000000010000000))>>7)*pow(2,-3)+ ((data & (0b0000000001000000))>>6)*pow(2,-4)+\
        ((data & (0b0000000000100000))>>5)*pow(2,-5)+ ((data & (0b0000000000010000))>>4)*pow(2,-6)+\
        ((data & (0b0000000000001000))>>3)*pow(2,-7)+ ((data & (0b0000000000000100))>>2)*pow(2,-8)+\
        ((data & (0b0000000000000010))>>1)*pow(2,-9)+ ((data & (0b0000000000000001)))*pow(2,-10);

        if (S==true){
            value = -1 * (float) pow(2,(double)E) * M;
        }
        if (S==false){
            value = 1 * (float) pow(2,(double)E) * M;
        }  
        return value;
    }



    int main()
    {
        printf("Hello World\n\n");

        int ii, jj, kk;
        uint8_t PackData[2];
        half HFData;
        float FData;

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
            printf("\nERROR in Setting port attributes");
        }
        else
        {
            printf("\nSERIAL Port Good to Go.\n");
        }

        // Flush Buffers
        tcflush(fid, TCIFLUSH);
        tcflush(fid, TCIOFLUSH);

        usleep(500000);   // 0.5 sec delay

        //--------------------------------------------------------------
        // RECEIVING BYTES - AND BUILD MESSAGE RECEIVED
        //--------------------------------------------------------------
        unsigned char rx_buffer[VMINX];
        unsigned char serial_message[NSERIAL_CHAR];
        bool          pickup = true;
        int           rx_length;
        int           nread = 0;   
        uint16_t      value = 0;
        half          hvalue;
        float         fvalue = 0; 

    	tcflush(fid, TCIOFLUSH);

        usleep(1000);   // .001 sec delay

        printf("Ready to receive message.\n");


        for (ii=0; ii<NSERIAL_CHAR; ii++)  serial_message[ii]=' ';

     
    	while (pickup && fid != -1)
    	{
            nread++;

            rx_length = read(fid, (void*)rx_buffer, VMINX);   // Filestream, buffer to store in, number of bytes to read (max)

            if(nread%2 !=0){
                value = (uint8_t) *rx_buffer;
                value = value << 8;
                //printf("value= %x\n", value);
            }
            else{
                value = value | (uint8_t) *rx_buffer;
                //printf("value= %x\n", value );
                //fvalue = changeToFloat(value);
                memcpy(&hvalue,&value,sizeof(value));
                fvalue = hvalue.operator float();
                //printf("value= %f \n", fvalue);                
                printf("value= %f \n", fvalue);
            }



            //printf("Event %d, rx_length=%d, Read=%x\n",  nread, rx_length, (uint8_t) *rx_buffer );

    		if (rx_length < 0)
    		{
    			//An error occured (will occur if there are no bytes)
    		}

    		if (rx_length == 0)
    		{
    			//No data waiting
    		}
    		
            if (rx_length>=0)
    		{
                if (nread<=NSERIAL_CHAR)   serial_message[nread-1] = rx_buffer[0];   // Build message 1 character at a time

                if (rx_buffer[0]=='#')   pickup=false;                               // # symbol is terminator
    		}
    	}

        printf("\nMessage Received: %s", serial_message);
     

        //-------------------------------------------
        //  CLOSE THE SERIAL PORT
        //-------------------------------------------
        close(fid);


        printf("\n\nGoodbye World\n\n");

    }