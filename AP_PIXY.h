

/*
 *      AP_PIXY.h  - Library for the CMUcam5 PIXY
 *
 *      code by Antoine LECESTRE for DIYdrones.com
 *      based on the arduino's TPixy.h library  form CMUCAM PIXY project
 *
 *      This is the main file containing Pixy's special functions. It should be called
 *      in files AP_PIXY_I2C.h or AP_PIXY_SPI.h depending on the type of connection
 *      you want to use.
 *
 */
 
 #ifndef __AP_PIXY_H__
#define __AP_PIXY_H__

#include <AP_HAL.h>
extern const AP_HAL::HAL& hal;

#define PIXY_INITIAL_ARRAYSIZE      30
#define PIXY_MAXIMUM_ARRAYSIZE      130
#define PIXY_START_WORD             0xaa55
#define PIXY_START_WORDX            0x55aa
#define PIXY_DEFAULT_ADDR           0x54  // I2C


// block structure, containing all the block's informations
struct Block 
{
    void print_block_info()
    {
       hal.console -> printf("sig: %d x: %d y: %d width: %d height: %d\n", (int)signature, (int)x, (int)y, (int)width, (int)height); 
    }
    uint16_t signature;
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
};

template <class LinkType> class AP_PIXY
{
public:

    // Constructor 
    AP_PIXY(uint8_t _addr=PIXY_DEFAULT_ADDR)
    {
        skipStart = false;
        blockCount = 0;
        // allocate memory
        blockArraySize = PIXY_INITIAL_ARRAYSIZE;
        blocks = new  Block[sizeof(Block)*blockArraySize];
        // init the connection port.
        link.init(_addr);
        // initialise health indicator
        PIXY_health = false;
    };
   ~AP_PIXY() // destructor
   {
        delete[] blocks;
    };
	
    
    // return blocks detected by PIXY  
    uint16_t getBlocks(uint16_t maxBlocks=1000);
  
    // heath
    bool  is_healthy();
  
    // block structure containing all the information regarding blocks : x, y , signature... 
    Block *blocks;
	
private:

    // function to identify beginning of information string sent by Pixy.
    bool getStart();
    // resize blocks buffer
    void resize();
    // health boolean
    bool  PIXY_health;

    // connection type : I2C or SPI for example
    LinkType link;
    // to skip getstart() identification
    bool skipStart;
    uint16_t blockCount;
    uint16_t blockArraySize;
};


#endif