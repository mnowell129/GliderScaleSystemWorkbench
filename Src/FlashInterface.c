#include "type.h"
#include "FreeRtosAll.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "FlashInterface.h"

#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_11   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_11  +  GetSectorSize(ADDR_FLASH_SECTOR_11) -1 /* End @ of user Flash area : sector start address + sector size -1 */


uint32_t HAL_GetTick(void)
{
   return(OSTIME());
}


/**
 * From ST hal example
 * 
 * @author Charles "Mickey" Nowell 
 * 
 * @param Address 
 * 
 * @return uint32_t 
 */



/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
uint32_t GetSector(uint32_t address)
{
   uint32_t sector = 0;
   if(address > ADDR_FLASH_END)
   {
      return(FLASH_SECTOR_15);
   }

   if((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0))
   {
      sector = FLASH_SECTOR_0;
   }
   else if((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1))
   {
      sector = FLASH_SECTOR_1;
   }
   else if((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2))
   {
      sector = FLASH_SECTOR_2;
   }
   else if((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3))
   {
      sector = FLASH_SECTOR_3;
   }
   else if((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4))
   {
      sector = FLASH_SECTOR_4;
   }
   else if((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5))
   {
      sector = FLASH_SECTOR_5;
   }
   else if((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6))
   {
      sector = FLASH_SECTOR_6;
   }
   else if((address < ADDR_FLASH_SECTOR_8) && (address >= ADDR_FLASH_SECTOR_7))
   {
      sector = FLASH_SECTOR_7;
   }
   else
   {
      // address is higher than sector 7, thus 128k byte blocks
      address &= ~0x1FFFF; // put on the sector boundary
                           // so address is now 0x08080000 for example
                           // subract the base address of sector 8
      address -= ADDR_FLASH_SECTOR_8;
      // now divide by 128k
      address >>= 17;
      // address is now the sector number offset from 8
      sector = address + 8;
   }
   return sector;
}

/**
  * @brief  Gets sector Size
  * @param  None
  * @retval The size of a given sector
  */
uint32_t GetSectorSize(uint32_t Sector)
{
   uint32_t sectorsize = 0x00;
   if((Sector == FLASH_SECTOR_0) || (Sector == FLASH_SECTOR_1) || (Sector == FLASH_SECTOR_2) || (Sector == FLASH_SECTOR_3))
   {
      sectorsize = 16 * 1024;
   }
   else if(Sector == FLASH_SECTOR_4)
   {
      sectorsize = 64 * 1024;
   }
   else
   {
      sectorsize = 128 * 1024;
   }
   return sectorsize;
}

/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;


bool flashUnlock(void)
{
   if((FLASH->CR & FLASH_CR_LOCK) != RESET)
   {
      /* Authorize the FLASH Registers access */
      FLASH->KEYR = FLASH_KEY1;
      FLASH->KEYR = FLASH_KEY2;
   }
   else
   {
      return(false);
   }

   return(true);
}


bool sectorErase(uint32_t sectorNumber)
{
   uint32_t SECTORError;
   // static so it will be all 0's
   static FLASH_OBProgramInitTypeDef options;
   HAL_FLASH_Unlock();

   /* Fill EraseInit structure*/
   EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
   EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_1; // was 3
   EraseInitStruct.Sector        = sectorNumber;
   EraseInitStruct.NbSectors     = 1;


   options.OptionType = OPTIONBYTE_WRP;
   options.Banks = 1;
   options.WRPSector = sectorNumber;
   options.WRPState = OB_WRPSTATE_DISABLE;
   HAL_FLASHEx_OBProgram(&options);
   /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
      you have to make sure that these data are rewritten before they are accessed during code
      execution. If this cannot be done safely, it is recommended to flush the caches by setting the
      DCRST and ICRST bits in the FLASH_CR register. */
   if(HAL_FLASHEx_Erase(&EraseInitStruct,&SECTORError) != HAL_OK)
   {
      /*
        Error occurred while sector erase.
        User can add here some code to deal with this error.
        SECTORError will contain the faulty sector and then to know the code error on this sector,
        user can call function 'HAL_FLASH_GetError()'
      */
      /* Infinite loop */
      HAL_FLASH_Lock();
      return(false);
   }
   HAL_FLASH_Lock();
   return(true);
}


bool sectorEraseByAddress(uint32_t sectorAddress)
{
   uint32_t sectorNumber;
   sectorNumber = GetSector(sectorAddress);
   return(sectorErase(sectorNumber));
}

bool flashWrite(uint32_t address, uint32_t data)
{
   HAL_FLASH_Unlock();
   if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,data) == HAL_OK)
   {
      HAL_FLASH_Lock();
      return(true);
   }
   else
   {
      HAL_FLASH_Lock();
      return(false);
   }
}

#define PARAMETER_SECTOR  FLASH_SECTOR_11
#define PARAMETER_ADDRESS ADDR_FLASH_SECTOR_11

#define K128_BYTES        (128 * 1024)

static uint8_t *currentWriteAddress;
bool   writeReady;
bool   flashIsOpenForRead;

bool prepareFlash(void)
{
   bool result;
   result = sectorErase(PARAMETER_SECTOR);
   if(result)
   {
      writeReady = true;
   }
   else
   {
      writeReady = false;
   }
   currentWriteAddress = (uint8_t *)(PARAMETER_ADDRESS);
   // currentWriteAddress += sizeof(ParameterHeaderType);
   // dataStartAddress = currentWriteAddress;
   return(result);
}
bool writeFlash(uint8_t *data,uint32_t length)
{
   if(!writeReady)
   {
      return(false);
   }
   HAL_FLASH_Unlock();
   while(length--)
   {
      if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,(uint32_t)(currentWriteAddress),*data++) != HAL_OK)
      {
         HAL_FLASH_Lock();
         return(false);
      }
      currentWriteAddress++;
   }
   HAL_FLASH_Lock();
   return(true);
}


