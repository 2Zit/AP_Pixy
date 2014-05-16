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
 *       AP_OpticalFlow_PIXY.cpp - PIXY used as OpticalFlow Library for
 *       Ardupilot Mega
 *       Code by Antoine LECESTRE. DIYDrones.com
 *
 */
 
#include <AP_HAL.h>
#include <AP_OpticalFlow_PIXY.h>


extern const AP_HAL::HAL& hal;

// Constructors ////////////////////////////////////////////////////////////////
AP_OpticalFlow_PIXY::AP_OpticalFlow_PIXY()
{
    horizontal_field_of_view = AP_OPTICALFLOW_PIXY_FOV;
    vertical_field_of_view = ((float)PIXY_PIXELS_RESOLUTION_Y / (float)PIXY_PIXELS_RESOLUTION_X) * horizontal_field_of_view;
    image_x_pos = (PIXY_PIXELS_RESOLUTION_X  / (float)2.0);
    image_y_pos = (PIXY_PIXELS_RESOLUTION_Y / (float)2.0);
    last_image_x_pos = image_x_pos;
    last_image_y_pos = image_y_pos;
}

// Public Methods //////////////////////////////////////////////////////////////
  
// initialise the sensor and check health of connection
void  AP_OpticalFlow_PIXY::init()
{
    //_flags.healthy = false;
    Pixy.getBlocks(); // try to make them talk to each others and use is to set connection health
    _flags.healthy = true;
    update_conversion_factors();   // set conversion factors
    
}

// read latest values from sensor and fill in x,y and totals
void AP_OpticalFlow_PIXY::update(void)
{
    uint8_t index;
// record index of good markers in a vector of uint8_t
    uint8_t good_blocks[detected_blocks]; // should be replace by vector<uint8_t> and use push_back after, but I couldn't use it!!!
    uint8_t number_of_good_blocks = 0;
    // blocks have been detected previously and stored in Pixy.blocks[i].
    for( index=0; index< (uint8_t)detected_blocks; index++)
    {
        // check for goo signature
        if(Pixy.blocks[index].signature == AP_OPTICALFLOW_PIXY_TARGET_SIGNATURE)
        {
            // We record in good_blocks[] the Pixy.blocks[] index of blocks with good signature
            good_blocks[number_of_good_blocks] = index;
            number_of_good_blocks++;
        }
    }
// We now know how many blocks with good signature have been detected, and we have record there position index in Pixy.blocks[]
    
    // if there is at least one valid block, the information is reliable
    if(number_of_good_blocks == 0)
        {
            surface_quality = 0; // make AP_OpticalFlow.cpp believe that surface quality is not good
        }else{
            surface_quality = 20; // make AP_OpticalFlow.cpp believe that surface quality is good enough
        }
        
// select the marker which is the closer to the center
    uint8_t  index_of_selected_marker = 0;
    // Maximum distance possible from the pixel in the centre of the image
    uint16_t max_dist_from_center_pixels = sqrt( pow((PIXY_PIXELS_RESOLUTION_X / (float)2.0), 2.0) + pow((PIXY_PIXELS_RESOLUTION_Y / (float)2.0), 2.0))
    // initialise distance minimal from the pixel in the centre of the image
    uint16_t min_dist_from_center_pixels = sqrt( pow((PIXY_PIXELS_RESOLUTION_X / (float)2.0), 2.0) + pow((PIXY_PIXELS_RESOLUTION_Y / (float)2.0), 2.0));
    for( index=0; index < number_of_good_blocks; index++)
    {
        // compute distance of the selected good block from the pixel in the centre of the image
        uint16_t distance_from_center_in_pixels = sqrt( pow((Pixy.blocks[good_blocks[index]].x - (PIXY_PIXELS_RESOLUTION_X / (float)2.0)), 2.0) + pow((Pixy.blocks[good_blocks[index]].y - (PIXY_PIXELS_RESOLUTION_Y / (float)2.0)), 2.0) );
        // compare it to the previous closest valid block from the pixel in the centre of the image
        if ( distance_from_center_in_pixels <= min_dist_from_center_pixels)
         {
            min_dist_from_center_pixels = distance_from_center_in_pixels;
           index_of_selected_marker = good_blocks[index]; 
        }
    }
        
// check for movement, update x,y values
    image_x_pos = Pixy.blocks[index_of_selected_marker].x ;
    image_y_pos = Pixy.blocks[index_of_selected_marker].y ;
    //debug
    //hal.console->printf("image_x_pos = %d        image_y_pos = %d    :\n", image_x_pos, image_y_pos );
  
    // raw sensor change in x and y position (i.e. unrotated)
    int16_t  raw_dx, raw_dy; 
    raw_dx = image_x_pos - last_image_x_pos;
    raw_dy = image_y_pos - last_image_y_pos;
    
    // record actual position for the next iteration.   
    last_image_x_pos = image_x_pos;
    last_image_y_pos = image_y_pos;
    
    // record time of last update.
    last_update = hal.scheduler->millis();
    
    // rotate dx and dy
    Vector3f rot_vector(raw_dx, raw_dy, 0);
    rot_vector.rotate(_orientation);
    dx = rot_vector.x;
    dy = rot_vector.y;

}



// parent method called at 50hz by periodic process
// update is slowed down to AP_OPTICALFLOW_PIXY_UPDATE_HZ and each instance's update function is called
//
void AP_OpticalFlow_PIXY::read()
{
    // update the detected blocks
    detected_blocks = Pixy.getBlocks();
    if (detected_blocks)
    {
        _num_calls++; // (global variable)
        if (_num_calls%(50 / AP_OPTICALFLOW_PIXY_UPDATE_HZ)==0) 
        {  
            update();
        }
    }
}

// clear_motion - will cause the Delta_X, Delta_Yto be cleared
void AP_OpticalFlow_PIXY::clear_motion()
{
    x_cm = 0;
    y_cm = 0;
    dx = 0;
    dy = 0;
}

// Private Methods //////////////////////////////////////////////////////////////

// updates conversion factors that are dependent upon field_of_view
void AP_OpticalFlow_PIXY::update_conversion_factors()
{
    // multiply this number by altitude and pixel change to get horizontal
    // move (in same units as altitude)
    //horizontal direction
    horizontal_conv_factor = ((1.0f / (float)(PIXY_PIXELS_RESOLUTION_X * AP_OPTICALFLOW_PIXY_SCALER))
                   * 2.0f * tanf(horizontal_field_of_view / 2.0f));
    horizontal_radians_to_pixels = (PIXY_PIXELS_RESOLUTION_X * AP_OPTICALFLOW_PIXY_SCALER) / horizontal_field_of_view;
    // vertical direction               
    vertical_conv_factor = ((1.0f / (float)(PIXY_PIXELS_RESOLUTION_Y * AP_OPTICALFLOW_PIXY_SCALER))
                   * 2.0f * tanf(vertical_field_of_view / 2.0f));               
    vertical_radians_to_pixels = (PIXY_PIXELS_RESOLUTION_Y * AP_OPTICALFLOW_PIXY_SCALER) / vertical_field_of_view;
}








