

/*
 *      AP_PIXY_I2C.h  - Library for the CMUcam5 PIXY
 *
 *      code by Antoine LECESTRE for DIYdrones.com
 *      based on the arduino's TPixy.h and PixyI2C.h library  form CMUCAM PIXY project
 *
 *      sensor should be connected to the I2C port
 *
 *
 */
 
#ifndef _PIXY_I2C_H
#define _PIXY_I2C_H

#include <AP_HAL.h>
extern const AP_HAL::HAL& hal;

#include <AP_PIXY.h>
#include <AP_PIXY.cpp>

class LinkI2C
{
public:
    // init - simply sets the i2c address
  void init(uint8_t address)
  {
    _addr = address;
  }
  
  // get a Word from PIXY
  uint16_t getWord()
  {
    uint16_t word_;
    uint8_t buff[2];
	uint8_t c;
    
    AP_HAL::Semaphore* _i2c_sem = hal.i2c->get_semaphore();
    if (!_i2c_sem->take(1)) {
     // the bus is busy - try again later
    return 0;
    }
    if ( hal.i2c->read(_addr, 2, buff) != 0) {
        {
        //PIXY_I2C_is_healthy = false; 
        _i2c_sem->give();
        return 0;
        }
    }else{
        c = buff[0];
        word_ = buff[1];
        word_ <<= 8;
        word_ |= c; 
        PIXY_I2C_is_healthy = true;
        _i2c_sem->give();
        return word_;
    }
  }
  
  // get a byte from PIXY
  uint8_t getByte()
  {
    uint8_t buff[1];
    AP_HAL::Semaphore* _i2c_sem = hal.i2c->get_semaphore();
    if (!_i2c_sem->take(1)) {
     // the bus is busy - try again later
    return 0;
    }
    if ( hal.i2c->read( _addr, 1, buff) != 0) 
        {
            //PIXY_I2C_is_healthy = false;
            _i2c_sem->give();
            return 0;
        }
    else
        {
            PIXY_I2C_is_healthy = true;
            _i2c_sem->give();
            return buff[0];
        }
  }
  
    // send health status
    bool health()
    {
        return PIXY_I2C_is_healthy;
    }
  
private:
    // health
    bool  PIXY_I2C_is_healthy;
 
    
  // address variable
    uint8_t _addr;
};

typedef AP_PIXY<LinkI2C> AP_PIXY_I2C;

#endif   //   _PIXY_I2C_H