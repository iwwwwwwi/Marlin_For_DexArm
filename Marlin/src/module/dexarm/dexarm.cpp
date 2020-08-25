/**
 * 
 * About Rotrics DexArm
 * Official Website:           https://rotrics.com
 * Rotrics Arm Diagram:        https://manual.rotrics.com/product-profile/rotrics-arm-diagram
 * DexArm's Coordinate System: https://manual.rotrics.com/product-profile/robot-arm-coordinate-system
 *
 */

#include "../../inc/MarlinConfig.h"
#include "dexarm.h"
#include "../planner.h"

int calibration_position_sensor_value[3]; //Set by M889
float x_axis_scaling_factor;			  //leveling
float y_axis_scaling_factor;			  //leveling
float front_module_offset = PEN_MODULE_OFFSET;

float start_angle_a = START_A_ANGLE;
float start_angle_b = START_B_ANGLE;
float start_angle_c = START_C_ANGLE;

float dexarm_offset = front_module_offset + OFFSET_ROT;

bool position_init_flag = false; //DexArm will not move without position init.
bool current_posution_flag = false;

float current_position_init[XYZE] = {START_X + dexarm_offset, START_Y, START_Z, 0.0};
#define CURRENT_POSITION_INIT                                            \
	if (current_posution_flag)                                           \
	{                                                                    \
		current_posution_flag = false;                                   \
		LOOP_XYZE(i) { current_position[i] = current_position_init[i]; } \
	}

void print_current_module_type()
{
	if (fabs(front_module_offset - PEN_MODULE_OFFSET) < 0.1)
	{
		MYSERIAL0.println("The current module is PEN");
	}

	if (fabs(front_module_offset - LASER_MODULE_OFFSET) < 0.1)
	{
		MYSERIAL0.println("The current module is LASER");
	}

	if (fabs(front_module_offset - PUMP_MODULE_OFFSET) < 0.1)
	{
		MYSERIAL0.println("The current module is PUMP");
	}

	if (fabs(front_module_offset - _3D_MODULE_OFFSET) < 0.1)
	{
		MYSERIAL0.println("The current module is 3D");
	}

	if (fabs(front_module_offset - CAMERA_MODULE_OFFSET) < 0.1)
	{
		MYSERIAL0.println("The current module is Camera");
	}
}

void module_position_init()
{
	MYSERIAL0.print("OFFSET_FRONT IS ");
	MYSERIAL0.println(front_module_offset);

	dexarm_offset = (front_module_offset + OFFSET_ROT);
	MYSERIAL0.print("dexarm_offset IS ");
	MYSERIAL0.println(dexarm_offset);
	print_current_module_type();
}

/**
 * get axis position sensor diff of target&current position
 */
int get_position_sensor_diff(int target_position, int current_position)
{
	int position_sensor_diff = 0;

	position_sensor_diff = current_position - target_position;
	if (position_sensor_diff > MAX_POSITION_SENSOR_DIFF)
	{
		position_sensor_diff -= 4096;
	}
	else if (position_sensor_diff < -1 * MAX_POSITION_SENSOR_DIFF)
	{
		position_sensor_diff += 4096;
	}

	return position_sensor_diff;
}

/**
 * Rotate angle of 3 Axis from current positon
 */
void rotate_angle_diff(abc_pos_t &angle_abc)
{
	xyz_pos_t pos = {0, 0, 0};
	abce_pos_t target;

	planner.set_machine_position_mm(pos);
	LOOP_XYZE(axis) { target[axis] = 0; }
	planner.buffer_segment(target, 30, active_extruder);

	LOOP_ABC(axis) { target[axis] = -1 * angle_abc[axis]; }
	planner.buffer_segment(target, 30, active_extruder);

	planner.set_machine_position_mm(pos);
	LOOP_ABCE(axis) { target[axis] = 0; }
	planner.buffer_segment(target, 30, active_extruder);
}

void get_current_encoder(){
	int current_position_sensor_value[3] = {0};
	LOOP_ABC(axis) { current_position_sensor_value[axis] = position_sensor_value_read(axis); }
	SERIAL_ECHOPAIR("M894 X", current_position_sensor_value[0], " Y", current_position_sensor_value[1], " Z", current_position_sensor_value[2]);
	SERIAL_ECHOPAIR("\r\n");
}

void process_encoder(int x, int y, int z){
	planner.synchronize(); 
	int current_position_sensor_value[3] = {0};
	int target_position_sensor_value[3] = {0};
	int diff_position_sensor_value[3] = {0};
	LOOP_ABC(axis) { current_position_sensor_value[axis] = position_sensor_value_read(axis); }
	target_position_sensor_value[0] = x;
	target_position_sensor_value[1] = y;
	target_position_sensor_value[2] = z;
	LOOP_ABC(axis) {
		diff_position_sensor_value[axis] = get_position_sensor_diff(target_position_sensor_value[axis], current_position_sensor_value[axis]);
	}

	abc_pos_t angle_diff;
	LOOP_ABC(axis)
	{
		angle_diff[axis] = ((float)diff_position_sensor_value[axis]) * 360.0 / 4096.0;
	}
	rotate_angle_diff(angle_diff);
	planner.synchronize();
}

/**
 * go to position M1111
 */
int position_M1111()
{
	int dif[3] = {0};
	int current_position_sensor_value[3];

	while (1)
	{
		LOOP_ABC(axis) { current_position_sensor_value[axis] = position_sensor_value_read(axis); }
		LOOP_ABC(axis)
		{
			dif[axis] = get_position_sensor_diff(calibration_position_sensor_value[axis], current_position_sensor_value[axis]);
		}

		if ((dif[0] == 0) & (dif[1] == 0) & (dif[2] == 0))
		{
			start_angle_a = START_A_ANGLE;
			start_angle_b = START_B_ANGLE;
			start_angle_c = START_C_ANGLE;
			current_position[0] = START_X + dexarm_offset;
			current_position[1] = START_Y;
			current_position[2] = START_Z;
			position_init_flag = true;
			return 1;
		}
		else
		{
			abc_pos_t angle_diff;
			LOOP_ABC(axis)
			{
				angle_diff[axis] = ((float)dif[axis]) * 360.0 / 4096.0;
				MYSERIAL0.print("angle diff is : ");
				MYSERIAL0.println(angle_diff[axis]);
			}
			rotate_angle_diff(angle_diff);
			planner.synchronize();
		}
	}
}

char inverse_kinematics_dexarm_xy_legace(const xyz_pos_t &position, abc_pos_t &angle)
{
	float x = position.x;
	float y = position.y;
	float z = position.z;

	//apply_leveling
	z += (x_axis_scaling_factor * x + y_axis_scaling_factor * (y - 200));

	float tmps = sqrt(x * x + y * y);

	x -= x * dexarm_offset / tmps;
	y -= y * dexarm_offset / tmps;

	float angleRot = 0;
	float angleLeft = 0;
	float angleRight = 0;

	if (x < 0.01)
	{
		x = 0.01;
	}

	// Calculate value of theta 1: the rotation angle
	if (y == 0)
	{
		angleRot = 90;
	}
	else
	{
		if (y < 0)
		{
			angleRot = -atan(x / y) * MATH_TRANS;
		}
		if (y > 0)
		{
			angleRot = 180 - atan(x / y) * MATH_TRANS;
		}
	}

	float s = sqrt(x * x + y * y);
	float d = sqrt(x * x + y * y + z * z);

	float thb1 = (acos(d / 2 / 150)) * MATH_TRANS;
	float thb2 = (atan(z / s)) * MATH_TRANS;

	angleLeft = thb1 + thb2;
	angleRight = 90.0 + (thb2 - thb1);

	angle[0] = angleRot;
	angle[1] = angleLeft;
	angle[2] = angleRight;

	return 0;
}

char inverse_kinematics_dexarm(const xyz_pos_t &position, abc_pos_t &angle)
{
	float x = position.x;
	float y = position.y;
	float z = position.z;

	//apply_leveling
	z += (x_axis_scaling_factor * x + y_axis_scaling_factor * (y - 200));

	float tmps = sqrt(x * x + y * y);

	x -= x * dexarm_offset / tmps;
	y -= y * dexarm_offset / tmps;

	float angleRot = 0;
	float angleLeft = 0;
	float angleRight = 0;

	if (y < 0.01)
	{
		y = 0.01;
	}

	// Calculate value of theta 1: the rotation angle
	if (x == 0)
	{
		angleRot = 90;
	}
	else
	{
		if (x < 0)
		{
			angleRot = -atan(y / x) * MATH_TRANS;
		}
		if (x > 0)
		{
			angleRot = 180 - atan(y / x) * MATH_TRANS;
		}
	}

	float s = sqrt(x * x + y * y);
	float d = sqrt(x * x + y * y + z * z);

	float thb1 = (acos(d / 2 / 150)) * MATH_TRANS;
	float thb2 = (atan(z / s)) * MATH_TRANS;

	angleLeft = thb1 + thb2;
	angleRight = 90.0 + (thb2 - thb1);

	angle[0] = angleRot;
	angle[1] = angleLeft;
	angle[2] = angleRight;
	return 0;
}

/**
 * go to position M1112
 */
int m1112_position(xyz_pos_t &position)
{
	int current_position_sensor_value[3] = {0};
	int dif[3] = {0};
	abc_pos_t target_angle;
	abc_pos_t diff_target_calibration_angle;

	int diff_position_sensor_value[3];
	int target_position_sensor_value[3] = {0};

	if (inverse_kinematics_dexarm(position, target_angle) == 0)
	{
		diff_target_calibration_angle[A_AXIS] = target_angle[A_AXIS] - START_A_ANGLE;
		//diff_target_calibration_angle[0] *= -1;
		diff_target_calibration_angle[B_AXIS] = START_B_ANGLE - target_angle[B_AXIS];
		diff_target_calibration_angle[C_AXIS] = target_angle[C_AXIS] - START_C_ANGLE;
	}
	LOOP_ABC(axis) { diff_position_sensor_value[axis] = (int)(diff_target_calibration_angle[axis] * 4096 / 360); }
	LOOP_ABC(axis)
	{
		target_position_sensor_value[axis] = calibration_position_sensor_value[axis] + diff_position_sensor_value[axis];
		if (target_position_sensor_value[axis] > 4096)
		{
			target_position_sensor_value[axis] -= 4096;
		}
		else if (target_position_sensor_value[axis] < 0)
		{
			target_position_sensor_value[axis] += 4096;
		}
	}

	while (1)
	{
		LOOP_ABC(axis) { current_position_sensor_value[axis] = position_sensor_value_read(axis); }
		LOOP_ABC(axis)
		{
			dif[axis] = get_position_sensor_diff(target_position_sensor_value[axis], current_position_sensor_value[axis]);
		}
		if ((dif[0] == 0) & (dif[1] == 0) & (dif[2] == 0))
		{
			start_angle_a = target_angle[0];
			start_angle_b = target_angle[1];
			start_angle_c = target_angle[2];
			LOOP_XYZ(axis) { current_position[axis] = position[axis]; }
			position_init_flag = true;
			return 1;
		}
		else
		{
			abc_pos_t angle_diff;
			LOOP_ABC(axis)
			{
				angle_diff[axis] = ((float)dif[axis]) * 360.0 / 4096.0;
			}
			rotate_angle_diff(angle_diff);
			planner.synchronize();
		}
	}
}

int m1113_position(xyz_pos_t &position)
{
	position_M1111();
	abc_pos_t target_angle;
	abc_pos_t diff_target_calibration_angle;
	LOOP_XYZ(axis) { target_angle[axis] = 1; }
	if (inverse_kinematics_dexarm(position, target_angle) == 0)
	{
		diff_target_calibration_angle[A_AXIS] = START_A_ANGLE - target_angle[A_AXIS];
		diff_target_calibration_angle[B_AXIS] = target_angle[B_AXIS] - START_B_ANGLE;
		diff_target_calibration_angle[C_AXIS] = START_C_ANGLE - target_angle[C_AXIS];
	}
	rotate_angle_diff(diff_target_calibration_angle);
	start_angle_a = target_angle[0];
	start_angle_b = target_angle[1];
	start_angle_c = target_angle[2];
	LOOP_XYZ(axis) { current_position[axis] = position[axis]; }
	position_init_flag = true;
	return 1;
}

void inverse_kinematics(const xyz_pos_t &raw)
{
	if (!position_init_flag)
	{
		MYSERIAL0.println("Send M1112 or click HOME to initialize DexArm first before any motion.");
		return;
	}
	abc_pos_t diff_angle;
	//if (!dexarm_position_is_reachable(raw))
	//{
	//LOOP_XYZE(i) { destination[i] = current_position[i]; }
	//return;
	//}
	CURRENT_POSITION_INIT;

	if (inverse_kinematics_dexarm(raw, diff_angle) == 0)
	{
		diff_angle[A_AXIS] -= start_angle_a;
		diff_angle[B_AXIS] = start_angle_b - diff_angle[B_AXIS];
		diff_angle[C_AXIS] -= start_angle_c;
	}
	//delta.set(diff_angle[0], diff_angle[1], diff_angle[2]);
}

void update_dexarm_offset(void)
{
	dexarm_offset = (front_module_offset + OFFSET_ROT);
}

bool dexarm_position_is_reachable(const xyz_pos_t &position)
{
	float x = position.x;
	float y = position.y;
	float z = position.z;

	int tmp_z = (int(z + 0.5) + abs(ZMINARM)) / 2 + 0.5;
	float tmps = sqrt(x * x + y * y) - dexarm_offset;

	if (z > ZMAXARM || z < ZMINARM)
	{
		MYSERIAL0.println("Z beyond limit.....");
		return false;
	}

	if (y < 0)
	{
		MYSERIAL0.println("Y beyond limit.....");
		return false;
	}

	if (tmps < ARMLIMIT[tmp_z * 2 + 1] + 1 || tmps > ARMLIMIT[tmp_z * 2] - 1)
	{
		MYSERIAL0.println("XY beyond limit.....");
		return false;
	}
	return true;
}