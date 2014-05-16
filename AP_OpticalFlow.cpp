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
 *       ADC.cpp - Analog Digital Converter Base Class for Ardupilot Mega
 *       Code by James Goppert. DIYDrones.com
 *
 */

#include "AP_OpticalFlow.h"

#define FORTYFIVE_DEGREES 0.78539816f

// set_orientation - Rotation vector to transform sensor readings to the body
// frame.
void AP_OpticalFlow::set_orientation(enum Rotation rotation)
{
    _orientation = rotation;
}

void AP_OpticalFlow::update()
{
}

void AP_OpticalFlow::init()
{
}

// updates internal lon and lat with estimation based on optical flow
void AP_OpticalFlow::update_position(float roll, float pitch, float sin_yaw, float cos_yaw, float altitude)
{
    float diff_roll     = roll  - _last_roll;
    float diff_pitch    = pitch - _last_pitch;
    float change_x, change_y;   // actual change in x, y coordinates

    // only update position if surface quality is good and angle is not
    // over 45 degrees
    if( surface_quality >= 10 && fabsf(roll) <= FORTYFIVE_DEGREES
     && fabsf(pitch) <= FORTYFIVE_DEGREES ) {
	    altitude = max(altitude, 0);

        // change in position is actual change measured by sensor (i.e. dx, dy) minus expected change due to change in roll, pitch
        #if OPTFLOW_PIXY == ENABLED // PIXY
            change_x = dx - (diff_roll * horizontal_radians_to_pixels);
            change_y = dy - (-diff_pitch * vertical_radians_to_pixels);
        #else   // ADNS3080
            change_x = dx - (diff_roll * radians_to_pixels);
            change_y = dy - (-diff_pitch * radians_to_pixels);
        #endif

        float avg_altitude = (altitude + _last_altitude)*0.5f;

        // convert raw change to horizontal movement in cm
        // perhaps this altitude should actually be the distance to the
        // ground?  i.e. if we are very rolled over it should be longer?
        #if OPTFLOW_PIXY == ENABLED // PIXY
                x_cm = -change_x * avg_altitude * horizontal_conv_factor;
        #else   // ADNS3080
               x_cm = -change_x * avg_altitude * conv_factor;
        #endif

        // for example if you are leaned over at 45 deg the ground will
        // appear farther away and motion from opt flow sensor will be less
        #if OPTFLOW_PIXY == ENABLED // PIXY
                y_cm = -change_y * avg_altitude * vertical_conv_factor;
        #else   // ADNS3080
                y_cm = -change_y * avg_altitude * conv_factor;
        #endif
    }

    _last_altitude = altitude;
    _last_roll = roll;
    _last_pitch = pitch;
}
