#ifndef __AP_OPTICALFLOW_H__
#define __AP_OPTICALFLOW_H__
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
 *       AP_OpticalFlow.cpp - OpticalFlow Base Class for Ardupilot Mega
 *       Code by Randy Mackay. DIYDrones.com
 *
 *       Methods:
 *               init() : initializate sensor and library.
 *               read   : reads latest value from OpticalFlow and
 *                        stores values in x,y, surface_quality parameter
 *               read_register()  : reads a value from the sensor (will be
 *                                  sensor specific)
 *               write_register() : writes a value to one of the sensor's
 *                                  register (will be sensor specific)
 */

#include <AP_Math.h>

class AP_OpticalFlow
{
public:
    // constructor
    AP_OpticalFlow() {
        _flags.healthy = false;
    };

    virtual void init();

    // healthy - return true if the sensor is healthy
    bool    healthy() const { return _flags.healthy; }
    
    // sets field of view of sensor
    void    set_field_of_view(const float fov) { field_of_view = fov; };

    // Rotation vector to transform sensor readings to the body frame.
    void    set_orientation(enum Rotation rotation);

    // read latest values from sensor and fill in x,y and totals.
    virtual void update();

    // updates internal lon and lat with estimation based on optical flow
    void    update_position(float roll, float pitch, float sin_yaw, float cos_yaw, float altitude);

    // public variables
    uint8_t  surface_quality;   // image quality (below 15 you can't trust the dx,dy values returned)
    int16_t  dx,dy;             // rotated change in x and y position
    uint32_t last_update;       // millis() time of last update
    
    // field of view in radians 
    #if OPTFLOW_PIXY == ENABLED
        float vertical_field_of_view;        // field of view is different in vertical and horizontal directions
        float horizontal_field_of_view;
    #endif
    float field_of_view;              // for ADNS3080

    
    // public variables for reporting purposes
    float    x_cm, y_cm;                    // x,y position in cm

protected:

    struct AP_OpticalFlow_Flags {
        uint8_t healthy     : 1;    // true if sensor is healthy
    } _flags;

    enum Rotation   _orientation;
    
    // COnversion factors  : multiply this number by altitude and pixel change to get horizontal move (in same units as altitude)
    #if OPTFLOW_PIXY == ENABLED
        //vertical (Y)
        float vertical_conv_factor;         // field of view is different in vertical and horizontal directions
        float vertical_radians_to_pixels;
        // horizontal (X)
        float horizontal_conv_factor; 
        float horizontal_radians_to_pixels;
    #endif
    float conv_factor;              // field of view is the same in both directions for ADNS3080
    float radians_to_pixels;
    
    
    

    float _last_roll;
    float _last_pitch;
    float _last_altitude;
};
    //#if OPTFLOW_PIXY == ENABLED
        // PIXY
        #include <AP_PIXY_I2C.h>
        #include <AP_OpticalFlow_PIXY.h>
    //#else
    //    // ADNS3080
    //    #include <AP_OpticalFlow_ADNS3080.h>
    //#endif

#endif
