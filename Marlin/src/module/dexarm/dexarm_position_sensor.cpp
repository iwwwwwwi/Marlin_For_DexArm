#include "../../inc/MarlinConfig.h"
#include <SlowSoftI2CMaster.h>  //https://github.com/stawel/SlowSoftI2CMaster

#define POSITION_SENSOR_NUM 3

const uint8_t sda_pins[POSITION_SENSOR_NUM] = {
  POSITION_SENSOR_I2C_SDA_X
    , POSITION_SENSOR_I2C_SDA_Y
      , POSITION_SENSOR_I2C_SDA_Z
};

const uint8_t sck_pins[POSITION_SENSOR_NUM] = {
  POSITION_SENSOR_I2C_SCK_X
    , POSITION_SENSOR_I2C_SCK_Y
      , POSITION_SENSOR_I2C_SCK_Z
};

static SlowSoftI2CMaster pots[POSITION_SENSOR_NUM] = {
  SlowSoftI2CMaster { sda_pins[X_AXIS], sck_pins[X_AXIS] }
    , SlowSoftI2CMaster { sda_pins[Y_AXIS], sck_pins[Y_AXIS] }
      , SlowSoftI2CMaster { sda_pins[Z_AXIS], sck_pins[Z_AXIS] }
};

static uint8_t i2c_read(const uint8_t channel, bool last, const uint8_t reg_address) {
  uint8_t value = 0;
  if (WITHIN(channel, 0, POSITION_SENSOR_NUM - 1)) {
    pots[channel].i2c_start(((POSITION_SENSOR_I2C_ADDRESS) << 1) | I2C_WRITE);
    pots[channel].i2c_write(reg_address);
    pots[channel].i2c_rep_start(((POSITION_SENSOR_I2C_ADDRESS) << 1) | I2C_READ);
    value = pots[channel].i2c_read(last);
    pots[channel].i2c_stop();
  }
  return value;
}

static void i2c_send(const uint8_t channel, const byte v) {
  if (WITHIN(channel, 0, DIGIPOT_I2C_NUM_CHANNELS - 1)) {
    pots[channel].i2c_start(((POSITION_SENSOR_I2C_ADDRESS) << 1) | I2C_WRITE);
    pots[channel].i2c_write(v);
    pots[channel].i2c_stop();
  }
}

word position_sensor_value_read(const uint8_t channel) {
  word value = i2c_read(channel, false, 0x0e);
	value <<= 8;
	value |= i2c_read(channel, true, 0x0f); 
	return value;
}

void position_sensor_init() {
  LOOP_L_N(i, POSITION_SENSOR_NUM)
    pots[i].i2c_init();
}