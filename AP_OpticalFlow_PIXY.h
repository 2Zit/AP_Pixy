#ifndef __AP_OPTICALFLOW_PIXY_H__
#define __AP_OPTICALFLOW_PIXY_H__
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_OpticalFlow_PIXY.h - PIXY used as OpticalFlow Library for
 *       Ardupilot Mega
 *       Code by Antoine LECESTRE. DIYDrones.com
 *
 *          TO DO : choose the type of connection you want to use (ie I2C or SPI)
 *                  choose the block's signature you want to identify
 *                  select sensor update frequency (up to 50hz)
 */
/* Connection : uncomment here what type of connection you want to use between
 arducopter and CMUcam PIXY. Don't forget to*/
        #define AP_OPTICALFLOW_PIXY_I2C         //  0  I2C    
        //#define AP_OPTICALFLOW_PIXY_SPI       //  1  SPI  NOT YET IMPLEMENTED!

// signature
        #define AP_OPTICALFLOW_PIXY_TARGET_SIGNATURE   1 
        //#define AP_OPTICALFLOW_PIXY_TARGET_SIGNATURE   2
        //#define AP_OPTICALFLOW_PIXY_TARGET_SIGNATURE   3
        //#define AP_OPTICALFLOW_PIXY_TARGET_SIGNATURE   4
        //#define AP_OPTICALFLOW_PIXY_TARGET_SIGNATURE   5 
        //#define AP_OPTICALFLOW_PIXY_TARGET_SIGNATURE   6
        //#define AP_OPTICALFLOW_PIXY_TARGET_SIGNATURE   7     

//FREQUENCY, (for information, PIXY send informations at 50hz)
        #define AP_OPTICALFLOW_PIXY_UPDATE_HZ         50            
//////////////////////////////////////////////////////////////////////////////
 

#include <AP_HAL.h>
#include "AP_OpticalFlow.h"
  
// PIXY hardware config
#define PIXY_PIXELS_RESOLUTION_X   320    //number of pixels in the horizontal field   ( 1/4 sensor with 1280 pixels in horizontal )
#define PIXY_PIXELS_RESOLUTION_Y   200     //number of pixels in the vertical field    ( 1/4 sensor with  800 pixels in horizontal  )
// field of view of Pixy sensor lenses
#define AP_OPTICALFLOW_PIXY_FOV 1.30899969f  //(rad)  = 75.0 degrees

// scaler - value returned when sensor is moved equivalent of 1 pixel
#define AP_OPTICALFLOW_PIXY_SCALER        1.0

// orientations for PIXY sensor
#define AP_OPTICALFLOW_PIXY_PINS_FORWARD ROTATION_YAW_180
#define AP_OPTICALFLOW_PIXY_PINS_FORWARD_RIGHT ROTATION_YAW_135
#define AP_OPTICALFLOW_PIXY_PINS_RIGHT ROTATION_YAW_90
#define AP_OPTICALFLOW_PIXY_PINS_BACK_RIGHT ROTATION_YAW_45
#define AP_OPTICALFLOW_PIXY_PINS_BACK ROTATION_NONE
#define AP_OPTICALFLOW_PIXY_PINS_BACK_LEFT ROTATION_YAW_315
#define AP_OPTICALFLOW_PIXY_PINS_LEFT ROTATION_YAW_270
#define AP_OPTICALFLOW_PIXY_PINS_FORWARD_LEFT ROTATION_YAW_225
  

class AP_OpticalFlow_PIXY : public AP_OpticalFlow
{
public:

    // constructor
    AP_OpticalFlow_PIXY();
    
    // initialise the sensor and check health of connection
    void    init();
    
    // read latest values from sensor and fill in x,y and totals,
    // returns true on successful read
    void    update(void);
    
    // called by timer process to read sensor data
    void    read();
    
    // will cause the x,y, dx, dy, and the sensor's motion registers to
    // be cleared
    void    clear_motion();
    
    
private:

    // update conversion factors based on field of view
    void update_conversion_factors();

    // detected blocks
    uint16_t detected_blocks;
    uint8_t _num_calls;     // used to throttle read down to 20hz

    // marker position on image
    int16_t image_x_pos, image_y_pos;
    int16_t last_image_x_pos, last_image_y_pos;
    
    #ifdef AP_OPTICALFLOW_PIXY_I2C
        // I2C PIXY device
        AP_PIXY_I2C  Pixy;
    #else
        #ifdef AP_OPTICALFLOW_PIXY_SPI
        // SPI PIXY device 
        AP_PIXY_SPI  Pixy;
        #endif
    #endif
};

#endif