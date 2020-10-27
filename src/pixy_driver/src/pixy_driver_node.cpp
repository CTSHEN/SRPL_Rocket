
#include <ros/ros.h>
#include <ros/time.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
//#include <string.h>
#include "i2c.h"
#include <pixy_msgs/PixyData.h>
#include <pixy_msgs/PixyBlock.h>

#define PIXY_ARRAYSIZE              10
#define PIXY_START_WORD             0xaa55
#define PIXY_START_WORD_CC          0xaa56
#define PIXY_START_WORDX            0x55aa
#define PIXY_SERVO_SYNC             0xff
#define PIXY_CAM_BRIGHTNESS_SYNC    0xfe
#define PIXY_LED_SYNC               0xfd
#define PIXY_OUTBUF_SIZE            64

#define PIXY_SYNC_BYTE              0x5a
#define PIXY_SYNC_BYTE_DATA         0x5b

I2CDevice device;

// the routines
void init();
int getStart(void);
uint16_t getBlocks(uint16_t maxBlocks);

std::string frame_id;


// data types
typedef enum
{
    NORMAL_BLOCK,
    CC_BLOCK // color code block
} BlockType;

typedef struct
{
  uint16_t signature;
  uint16_t x;
  uint16_t y;
  uint16_t width;
  uint16_t height;
  uint16_t angle; // angle is only available for color coded blocks
  void reset(void){
      signature = 0;
      x = 0;
      y = 0;
      width = 0;
      height = 0;
      angle = 0;
  }
} Block;

// communication routines
static uint16_t getWord(void);
//static int send(uint8_t *data, int len);

extern uint8_t getByte(void);
extern int sendByte(uint8_t byte);

uint8_t getByte(void)
{
    unsigned char buffer;
    i2c_ioctl_read(&device, 0x0, &buffer, sizeof(buffer)); // TODO

    return uint8_t(buffer);
}
uint16_t getWord(void)
{
  // this routine assumes little endian
  uint16_t w;
  uint8_t c;
  c = getByte();
  w = getByte();
  w <<= 8;
  w |= c;
  return w;
}

/*int send(uint8_t *data, int len)
{
  int i;
  for (i=0; i<len; i++)
    sendByte(data[i]);

  return len;
}*/

static int g_skipStart = 0;
static BlockType g_blockType;
static Block *g_blocks;

void init()
{
  g_blocks = (Block *)malloc(sizeof(Block)*PIXY_ARRAYSIZE);
}

int getStart(void)
{
  uint16_t w, lastw;

  lastw = 0xffff;

  while(1)
  {
    w = getWord();
    if (w==0 && lastw==0)
      return 0; // no start code
    else if (w==PIXY_START_WORD && lastw==PIXY_START_WORD)
    {
      g_blockType = NORMAL_BLOCK;
      return 1; // code found!
    }
    else if (w==PIXY_START_WORD_CC && lastw==PIXY_START_WORD)
    {
      g_blockType = CC_BLOCK; // found color code block
      return 1;
    }
    else if (w==PIXY_START_WORDX)
    getByte(); // we're out of sync! (backwards)

  lastw = w;
  }
}

uint16_t getBlocks(uint16_t maxBlocks)
{
  uint8_t i;
  uint16_t w, blockCount, checksum, sum;
  Block *block;

  if (!g_skipStart)
  {
    if (getStart()==0)
      return 0;
  }
  else
    g_skipStart = 0;

  for(blockCount=0; blockCount<maxBlocks && blockCount<PIXY_ARRAYSIZE;)
  {
    checksum = getWord();
    if (checksum==PIXY_START_WORD) // we've reached the beginning of the next frame
    {
      g_skipStart = 1;
      g_blockType = NORMAL_BLOCK;
      return blockCount;
    }
    else if (checksum==PIXY_START_WORD_CC)
    {
      g_skipStart = 1;
      g_blockType = CC_BLOCK;
      return blockCount;
    }
    else if (checksum==0)
      return blockCount;

    block = g_blocks + blockCount;

    for (i=0, sum=0; i<sizeof(Block)/sizeof(uint16_t); i++)
    {
      if (g_blockType==NORMAL_BLOCK && i>=5) // no angle for normal block
      {
        block->angle = 0;
        break;
      }
      w = getWord();
      sum += w;
      *((uint16_t *)block + i) = w;
    }

    // check checksum
    if (checksum==sum)
      blockCount++;
    else
      printf("checksum error!\n");

    w = getWord();
    if (w==PIXY_START_WORD)
      g_blockType = NORMAL_BLOCK;
    else if (w==PIXY_START_WORD_CC)
      g_blockType = CC_BLOCK;
    else
      return blockCount;
  }
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "pixy_driver");

    ros::NodeHandle n;
	ROS_INFO("Pixy Driver Node Start");
    // get parameters
    
    int i2c_addr;
    n.param("i2c_addr",i2c_addr,0x54);
    n.param<std::string>(std::string("frame_id"), frame_id,
			std::string("pixy_frame"));
    ros::Publisher publisher_ = n.advertise<pixy_msgs::PixyData>("block_data", 1); //CTSHEN

	ros::Rate loop_rate(50);  // CTSHEN  
	
	int fd;
    //I2CDevice device;
    const char *data = "9876543";
    uint16_t blockCount;
	//unsigned char buffer[14];
	//ssize_t input_size = sizeof(buffer);
	//memset(buffer, 0, sizeof(buffer));

    /* First open i2c bus */
    if ((fd = i2c_open("/dev/i2c-0")) == -1) {

        perror("Open i2c bus error");
        return -1;
    }

    /* Fill i2c device struct */
    device.bus = fd;
    device.addr = i2c_addr;
    device.tenbit = 0;
    device.delay = 10;
    device.flags = 0;
    device.page_bytes = 8;
    device.iaddr_bytes = 0; /* Set this to zero, and using i2c_ioctl_xxxx API will ignore chip internal address */

    init(); // start pixy process

    /* Write data to i2c */
    //if (i2c_ioctl_write(&device, 0x0, data, strlen(data)) != strlen(data)) {

        /* Error process */
    //}
	/*if( (i2c_ioctl_read(&device, 0x0, buffer, input_size)) != input_size){
		perror("sth wrong!");
		return -1;
	}*/
    pixy_msgs::PixyData block_data;

	while(ros::ok){ //TODO
		
        blockCount = getBlocks(1);
        block_data.blocks.clear();
        block_data.header.stamp = ros::Time::now();
        // Assume only one block
        pixy_msgs::PixyBlock pixy_block;
        pixy_block.signature = g_blocks[0].signature;
        pixy_block.roi.x_offset = g_blocks[0].x;
        pixy_block.roi.y_offset = g_blocks[0].y;
        pixy_block.roi.height = g_blocks[0].height;
        pixy_block.roi.height = g_blocks[0].width;
        pixy_block.roi.do_rectify = false;

        block_data.blocks.push_back(pixy_block);

        //publish the message
        publisher_.publish(block_data);


        printf(" Sig: %" PRIu16 ",x: %" PRIu16 ",y: %" PRIu16 "\n", g_blocks[0].signature, g_blocks[0].x, g_blocks[0].y);
        printf("blockCount =%" PRId16 "\n", blockCount);
        g_blocks->reset();

        ros::spinOnce();
        loop_rate.sleep();
	}


    i2c_close(fd);
    return 0;

}

