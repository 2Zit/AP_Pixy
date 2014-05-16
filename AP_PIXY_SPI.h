

/*
 *      AP_PIXY_SPI.h  - Library for the CMUcam5 PIXY
 *
 *      code by Antoine LECESTRE for DIYdrones.com
 *      based on the arduino's TPixy.h and Pixy.h library  form CMUCAM PIXY project
 *
 *      sensor should be connected to the SPI port
 *
 *
 */
 
/* /////////////////////////////////////////////////////////////////////////////////////////
*
*       This is not yet implemented !
*
*/////////////////////////////////////////////////////////////////////////////////////////
 
 
 
 
#ifndef _PIXY_SPI_H
#define _PIXY_SPI_H

#include <AP_HAL.h>
extern const AP_HAL::HAL& hal;

#include <AP_PIXY.h>
#include <AP_PIXY.cpp>

class LinkSPI
{
public:
    // init - simply sets the SPI address
  void init(uint8_t address)
  {
  }
  
  // get a Word from PIXY
  uint16_t getWord()
  {
  }
  
  // get a byte from PIXY
  uint8_t getByte()
  {
  }
  
  // send health status
  bool health()
  {
      return PIXY_SPI_is_healthy;
  }
  
private:
  // health
  bool  PIXY_SPI_is_healthy;
 
};

typedef AP_PIXY<LinkSPI> AP_PIXY_SPI;

#endif   //   _PIXY_SPI_H