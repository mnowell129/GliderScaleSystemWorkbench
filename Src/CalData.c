/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>

#include "type.h"
#include "FreeRtosAll.h"
#include "flashinterface.h"
#include "crc.h"
#include "CalData.h"


static bool calIsValid = false;

static SystemCalDataType localCalData =
{
   .scaleCalibrationData = 
   {
      // non zero values to make the math not crash.
      // these get updated and written into
      // flash in calibration
      .frontZero = 0.0f,
      .backZero  = 0.0f,
      .frontAtCal = 1000.0f, // made up
      .backAtCal =  1000.0f, // made up
      .frontSlope = 1.0f,  // made up
      .backSlope = 1.0f,   // made up
      .calWeight = 500.00f, // made up
      .WingPegDist = 120.0f, // some nominal value
      .LEstopperDist = 30.0f, // some nominal value
      .frontGain  = 3,
      .backGain = 3
   }
};
/**
 * Recover cal data from flash. 
 * Check the crc. 
 * 
 * @author Charles "Mickey" Nowell (3/26/2018)
 * 
 * @param data 
 * @param length 
 * 
 * @return bool 
 */
bool readCalData(uint8_t *data,uint32_t length)
{
   if(calcCRC((uint8_t *)ADDR_FLASH_SECTOR_11,length + 2) == 0)
   {
      memcpy(data,(uint8_t *)ADDR_FLASH_SECTOR_11,length);
      calIsValid = true;
      return(true);
   }
   calIsValid = false;
   return(false);
}

/**
 * Write the cal data to flash. 
 * Append a crc. 
 * 
 * @author Charles "Mickey" Nowell (3/26/2018)
 * 
 * @param data 
 * @param length 
 */
void writeCalData(uint8_t const *data,uint32_t length)
{
   uint16_t  crc;
   uint8_t   highByte,lowByte;
   crc = calcCRC((uint8_t *)data,length);
   prepareFlash();
   writeFlash((uint8_t *)data,length);
   highByte = crc >> 8;
   lowByte = crc & 0xff;
   writeFlash((uint8_t *)&highByte,1);
   writeFlash((uint8_t *)&lowByte,1);

}

/**
 * Recover the flash data. 
 * Indicate pass if crc is good. 
 * 
 * @author Charles "Mickey" Nowell (3/26/2018)
 * 
 * @param void 
 * 
 * @return bool 
 */
bool loadCalFromFlash(void)
{
   return(readCalData((uint8_t*)&localCalData,sizeof(SystemCalDataType)));
}
/**
 * Push the local ram copy to flash. 
 * Puts the crc on as well. 
 * Takes a second or two because it 
 * erases the flash sector. 
 * 
 * @author Charles "Mickey" Nowell (3/26/2018)
 * 
 * @param void 
 */
void pushCalToFlash(void)
{
   writeCalData((uint8_t*)&localCalData,sizeof(SystemCalDataType));
}

/**
 * Update the private ram copy of cal data 
 * with screen values. 
 * 
 * @author Charles "Mickey" Nowell (3/26/2018)
 * 
 * @param scaleCalibration 
 */
void updateScaleCalibration(ScaleCalibrationPtr scaleCalibration)
{
   if(!scaleCalibration)
   {
      return;
   }
   memcpy((uint8_t*)(&localCalData.scaleCalibrationData),(uint8_t *)scaleCalibration,sizeof(ScaleCalibrationType));
}
/**
 * Update the working copy of cal data from 
 * the local ram shadow copy of flash. 
 * Only if it's good. 
 * 
 * @author Charles "Mickey" Nowell (3/26/2018)
 * 
 * @param scaleCalibration 
 * 
 * @return bool 
 */
bool loadScaleCalibration(ScaleCalibrationPtr scaleCalibration)
{
   if(!scaleCalibration)
   {
      return(false);
   }
   if(calIsValid)
   {
      memcpy((uint8_t *)scaleCalibration,(uint8_t*)(&localCalData.scaleCalibrationData),sizeof(ScaleCalibrationType));
      return(true);
   }
   return(false);
}
/**
 * Update local ram copy of TS cal data. 
 *  
 * 
 * @author Charles "Mickey" Nowell (3/26/2018)
 * 
 * @param tsCalibration 
 */
void updateTSCalibration(TSCalibrationPtr tsCalibration)
{
   if(!tsCalibration)
   {
      return;
   }
   memcpy((uint8_t*)(&localCalData.tsCalibrationData),(uint8_t *)tsCalibration,sizeof(TSCalibrationType));
}
/**
 * Update working copy of TS cal data.
 * 
 * @author Charles "Mickey" Nowell (3/26/2018)
 * 
 * @param tsCalibration 
 * 
 * @return bool 
 */
bool loadTSCalibration(TSCalibrationPtr tsCalibration)
{
   if(!tsCalibration)
   {
      return(false);
   }
   if(calIsValid)
   {
      memcpy((uint8_t *)tsCalibration,(uint8_t*)(&localCalData.tsCalibrationData),sizeof(TSCalibrationType));
      return(true);
   }
   return(false);
}

