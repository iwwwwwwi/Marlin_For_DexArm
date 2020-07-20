#pragma once

#include "dexarm_position_config.h"
#include "dexarm_position_reachable.h"
#include "dexarm_position_sensor.h"
#include "dexarm_update_manager.h"

//Dexarm config
extern int calibration_position_sensor_value[3];
extern float x_axis_scaling_factor;
extern float y_axis_scaling_factor;
extern float front_module_offset;
extern float delta_segments_per_second;

extern bool position_init_flag;

void print_current_module_type();
void module_position_init();
void update_dexarm_offset(void);

void get_current_encoder();
void process_encoder(int x, int y, int z);

int position_M1111();
int m1112_position(xyz_pos_t &position);
int m1113_position(xyz_pos_t &position);

void rotate_angle_diff(abc_pos_t &angle_abc);
void inverse_kinematics(const xyz_pos_t &raw);

bool dexarm_position_is_reachable(const xyz_pos_t &position);

