#pragma once

#define NEED_UPDATE_FLAG_ADDRESS   ((uint32_t)0x0800CFF0)
#define NEED_VERIFICATION_FLAG_ADDRESS   ((uint32_t)(NEED_UPDATE_FLAG_ADDRESS - 4))
#define DEVICE_UUID_ADDRESS   ((uint32_t)(NEED_VERIFICATION_FLAG_ADDRESS - 4))
#define NEED_UPDATE_DATE_ADDRESS   ((uint32_t)(DEVICE_UUID_ADDRESS - 4))


#define EEPROM_VSERION   ((uint32_t)(0x080FC060 + 4))
#define AS5600_ADDRESS1   ((uint32_t)(0x080FC2D8))
#define AS5600_ADDRESS1   ((uint32_t)(0x080FC2D8))
#define AS5600_ADDRESS2   ((uint32_t)(0x080FC2D8 + 4))
#define AS5600_ADDRESS3   ((uint32_t)(0x080FC2D8 + 8))

/* Need Flag */
typedef enum {
  NO_NEED   = 0U,
  NEED,
}Need_FlagTypeDef;

void enter_update(void);
void check_update_flag(void);