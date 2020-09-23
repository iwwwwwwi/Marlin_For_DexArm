#pragma once

int position_sensor_value_read(const uint8_t channel);
void position_sensor_init();
void check_position_sensor(const int sensor_value[]);