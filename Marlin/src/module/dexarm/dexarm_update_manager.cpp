#include "../../MarlinCore.h"
#include "../../core/debug_out.h"

#include "stm32f4xx_hal.h"
#include "../configuration_store.h"
#include "dexarm.h"

#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_3   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_11  +  GetSectorSize(ADDR_FLASH_SECTOR_11) -1 /* End @ of user Flash area : sector start address + sector size -1 */
#define NEED_UPDATE                ((uint32_t)0x00000001)
#define NEED_VERIFICATION                 ((uint32_t)0x00000002)
#define DATA_32                 ((uint32_t)0x12345678)

uint32_t FirstSector = 0, NbOfSectors = 0, Address = 0;
uint32_t SectorError = 0;
__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;
int time = 200;
uint32_t device_id = 0;

static FLASH_EraseInitTypeDef EraseInitStruct;

static uint32_t GetSector(uint32_t Address);

void enter_update(void)
{
  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Erase the user Flash area
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

  /* Get the 1st sector to erase */
  FirstSector = GetSector(FLASH_USER_START_ADDR);
  /* Get the number of sector to erase from 1st sector*/
  NbOfSectors = 1;//GetSector(FLASH_USER_END_ADDR) - FirstSector + 1;

  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FirstSector;
  EraseInitStruct.NbSectors = NbOfSectors;
  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  {
    /*
      Error occurred while sector erase.
      User can add here some code to deal with this error.
      SectorError will contain the faulty sector and then to know the code error on this sector,
      user can call function 'HAL_FLASH_GetError()'
    */
    /*
      FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
    */
    //Error_Handler();
  }

  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
     you have to make sure that these data are rewritten before they are accessed during code
     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
     DCRST and ICRST bits in the FLASH_CR register. */
  __HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

  __HAL_FLASH_DATA_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_RESET();

  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();

	/* Program the user Flash area word by word
    	    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, NEED_UPDATE_FLAG_ADDRESS, NEED_UPDATE) == HAL_OK)
	{
		//Address = Address - 4;
	}
	else
	{
		/* Error occurred while writing data in Flash memory.
    	         User can add here some code to deal with this error */
		/*
    	        FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
    	      */
		//Error_Handler();
	}

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, NEED_VERIFICATION_FLAG_ADDRESS, NEED_VERIFICATION) == HAL_OK)
	{
		//Address = Address - 4;
	}
	else
	{
		/* Error occurred while writing data in Flash memory.
    	         User can add here some code to deal with this error */
		/*
    	        FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
    	      */
		//Error_Handler();
	}

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, DEVICE_UUID_ADDRESS, device_id) == HAL_OK)
	{
		//Address = Address - 4;
	}
	else
	{
		/* Error occurred while writing data in Flash memory.
    	         User can add here some code to deal with this error */
		/*
    	        FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
    	      */
		//Error_Handler();
	}
	/* Lock the Flash to disable the flash control register access (recommended
    	     to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();
  NVIC_SystemReset();  
}

void reset_calibration_position_sensor_value(){
    calibration_position_sensor_value[0] = *(volatile uint32_t *)AS5600_ADDRESS1;
    calibration_position_sensor_value[1] = *(volatile uint32_t *)AS5600_ADDRESS2;
    calibration_position_sensor_value[2] = *(volatile uint32_t *)AS5600_ADDRESS3;

    (void)settings.save();
}

void check_eeprom_version()
{
    volatile uint32_t eeprom_version = 0;
    eeprom_version = *(volatile uint32_t *)EEPROM_VSERION;
    //eeprom version V74
    if(eeprom_version == 3422038){
      reset_calibration_position_sensor_value();
    }
}

int get_need_update_flag()
{
    volatile int need_update_flag = 0;
    need_update_flag = *(volatile int *)NEED_UPDATE_FLAG_ADDRESS;
    return need_update_flag;
}

uint32_t get_device_id()
{
    volatile uint32_t device_id = 0;
    device_id = *(volatile uint32_t *)DEVICE_UUID_ADDRESS;
    return device_id;
}

void clear_need_verification(void){
  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Erase the user Flash area
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

  /* Get the 1st sector to erase */
  FirstSector = GetSector(FLASH_USER_START_ADDR);
  /* Get the number of sector to erase from 1st sector*/
  NbOfSectors = 1;//GetSector(FLASH_USER_END_ADDR) - FirstSector + 1;

  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FirstSector;
  EraseInitStruct.NbSectors = NbOfSectors;
  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  {
    /*
      Error occurred while sector erase.
      User can add here some code to deal with this error.
      SectorError will contain the faulty sector and then to know the code error on this sector,
      user can call function 'HAL_FLASH_GetError()'
    */
    /*
      FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
    */
    //Error_Handler();
  }

  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
     you have to make sure that these data are rewritten before they are accessed during code
     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
     DCRST and ICRST bits in the FLASH_CR register. */
  __HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

  __HAL_FLASH_DATA_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_RESET();

  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();

	//if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, DATA_32) == HAL_OK)
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, DEVICE_UUID_ADDRESS, device_id) == HAL_OK)
	{
		//Address = Address - 4;
	}
	else
	{
		/* Error occurred while writing data in Flash memory.
    	         User can add here some code to deal with this error */
		/*
    	        FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
    	      */
		//Error_Handler();
	}

  HAL_FLASH_Lock(); 
}

void check_update_flag(void)
{
  check_eeprom_version();
  device_id = get_device_id();
  if (get_need_update_flag() == NEED)
  {
    clear_need_verification();
  }
}

/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_SECTOR_7;
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_SECTOR_8;
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_SECTOR_9;
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_SECTOR_10;
  }
  else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
  {
    sector = FLASH_SECTOR_11;
  }

  return sector;
}