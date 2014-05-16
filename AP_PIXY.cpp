
/*
 *      AP_PIXY.h  - Library for the CMUcam5 PIXY
 *
 *      code by Antoine LECESTRE for DIYdrones.com
 *      based on the arduino's TPixy.h, Pixy.h and PixyI2C.h library  form CMUCAM PIXY project
 *
 *      sensor should be connected to the I2C or SPI depending on connection choice. 
 *
 *       Variables:
 *               bool PIXY_health : indicates whether last communication with sensor was successful
 */
 
 
 // AVR LibC Includes
#include <AP_PIXY.h>
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;


// private Methods //////////////////////////////////////

//identify beginning of information string sent by Pixy.
template <class LinkType> bool AP_PIXY<LinkType>::getStart()
{
  uint16_t word_, last_word;
  last_word = 0xffff;
  
  while(true)
  { 
    //get a word from Pixy
    word_ = link.getWord();
    
    // check link health
    PIXY_health = link.health();

    if (word_ == 0 && last_word == 0)
	{
      // no word received
      hal.scheduler->delay_microseconds(10);
	  return false;
	}		
    else if (word_==PIXY_START_WORD && last_word==PIXY_START_WORD)
      // this is PIXY_START_WORD
      return true;
	else if (word_==PIXY_START_WORDX)
	{
	  hal.console->printf("reorder");
	  link.getByte(); // resync
      PIXY_health = link.health();
	}
	last_word = word_; 
  }
}

// resize memory allocation
template <class LinkType> void AP_PIXY<LinkType>::resize()
{
  Block *newBlocks;
  blockArraySize += PIXY_INITIAL_ARRAYSIZE;
  newBlocks = new  Block[sizeof(Block)*blockArraySize];
  memcpy(newBlocks, blocks, sizeof(Block)*blockCount);
  delete[] blocks;
  blocks = newBlocks;
}  


// Public Methods /////////////////////////////////////////////////////////////

// ask Pixy for the detected blocks
template <class LinkType> uint16_t AP_PIXY<LinkType>::getBlocks(uint16_t maxBlocks)
{
  uint8_t index;
  uint16_t word_, checksum, sum;
  Block *block;
  
  if (!skipStart)
    {if (getStart()==false)
        return 0; 
    }
  else
      {  skipStart = false;}
	
  for(blockCount=0; blockCount<maxBlocks && blockCount<PIXY_MAXIMUM_ARRAYSIZE;)
  {
    checksum = link.getWord();
    PIXY_health = link.health();
    if (checksum==PIXY_START_WORD) // we've reached the beginning of the next frame
    {
      skipStart = true;
	  //Serial.println("skip");
      return blockCount;
    }
    else if (checksum==0)
    {
      return blockCount;
    }
    
	if (blockCount>blockArraySize)
		resize();
	
	block = blocks + blockCount;
	
    for (index=0, sum=0; index<sizeof(Block)/sizeof(uint16_t); index++)
    {
      word_ = link.getWord();
      PIXY_health = link.health();
      sum += word_;
      *((uint16_t *)block + index) = word_;
    }

    if (checksum==sum)
      blockCount++;
	
	word_ = link.getWord();
    PIXY_health = link.health();
    if (word_!=PIXY_START_WORD)
       {
        return blockCount;
       }
  }
  return 0;
}

// return pixy's heath status
template <class LinkType> bool AP_PIXY<LinkType>::is_healthy()
{
    return PIXY_health;
}



