/*
 *  Example of AP_OpticalFlow PIXY library.
 *  Code by Antoine LECESTRE, for DIYdrones.
 */

// define printf update frequency
#define FREQ_UPDATE_HZ   3 //HZ

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_PIXY_I2C.h>

// I2C pixy
AP_PIXY_I2C Pixy;

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;


void setup()
{
    hal.console->println("ArduPilot Mega OpticalFlow Pixy library test ver 1.0");

    hal.scheduler->delay(1000);

}

void loop()
{
  static uint8_t i = 0;
  uint16_t blocks;
  uint8_t index;
  
  blocks = Pixy.getBlocks();
  if(Pixy.is_healthy())
  {
    if (blocks)
    {
     i++;
      if (i%(50 / FREQ_UPDATE_HZ)==0)
      {
         hal.console->printf("Detected %d:\n", blocks );
         hal.console->println("   ");
         for( index=0; index< blocks; index++)
           {
             hal.console->printf("  block %d:\n", index + 1);
             Pixy.blocks[index].print_block_info();
             hal.console->println("   ");
           }
       }
     }
  }else{
        hal.console->println(" CONNECTION ERROR " ); 
  }
}

AP_HAL_MAIN();
